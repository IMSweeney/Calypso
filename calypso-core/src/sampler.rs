//! 100 Hz IMU sampling: fixed-point sample format and a RAM ring buffer.
//!
//! Platform-independent; the firmware task feeds it from the BNO055 and the
//! rep detector consumes windows from it. See `docs/REP_TRACKING.md`.

/// Linear-acceleration scale: LSB per m/s² (matches the BNO055's native
/// fixed-point output, so no conversion is needed on the device).
pub const ACCEL_SCALE: i32 = 100;

/// Quaternion scale: LSB per unit quaternion component (matches the BNO055's
/// native 1/2^14 output).
pub const QUAT_SCALE: i32 = 1 << 14;

/// Sample rate of the sampler task.
pub const SAMPLE_RATE_HZ: u32 = 100;

/// One IMU sample, fixed point (20 bytes in memory: 18 + alignment padding).
///
/// - `t_ms`: monotonic timestamp in milliseconds.
/// - `accel`: linear (gravity-removed) acceleration, `ACCEL_SCALE` LSB/m/s².
/// - `quat`: orientation quaternion `[w, x, y, z]`, `QUAT_SCALE` LSB/unit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Sample {
    pub t_ms: u32,
    pub accel: [i16; 3],
    pub quat: [i16; 4],
}

impl Sample {
    pub const BYTE_SIZE: usize = 20;

    /// All-zero sample (const-constructible for static ring initialisation).
    pub const ZERO: Sample = Sample {
        t_ms: 0,
        accel: [0; 3],
        quat: [0; 4],
    };

    /// Build from floating-point readings (m/s², unit quaternion).
    pub fn from_f32(t_ms: u32, accel: [f32; 3], quat_wxyz: [f32; 4]) -> Self {
        Sample {
            t_ms,
            accel: accel.map(|a| quantize(a, ACCEL_SCALE)),
            quat: quat_wxyz.map(|q| quantize(q, QUAT_SCALE)),
        }
    }

    /// Linear acceleration in m/s².
    pub fn accel_f32(&self) -> [f32; 3] {
        self.accel.map(|a| a as f32 / ACCEL_SCALE as f32)
    }

    /// Unit quaternion `[w, x, y, z]`.
    pub fn quat_f32(&self) -> [f32; 4] {
        self.quat.map(|q| q as f32 / QUAT_SCALE as f32)
    }
}

/// Round-and-clamp a float to i16 fixed point (no_std: no f32::round).
pub fn quantize(v: f32, scale: i32) -> i16 {
    let scaled = v * scale as f32;
    let rounded = if scaled >= 0.0 {
        scaled + 0.5
    } else {
        scaled - 0.5
    };
    if rounded >= i16::MAX as f32 {
        i16::MAX
    } else if rounded <= i16::MIN as f32 {
        i16::MIN
    } else {
        rounded as i16
    }
}

/// Fixed-capacity ring of samples, oldest dropped on overflow.
///
/// Borrows its buffer rather than owning it, like [`crate::rep::TraceBuf`] does
/// for traces: a ring that owned a 30 s window would be 54 KB, and building that
/// as a value means 54 KB on the stack before it can be moved to the heap — more
/// than twice the firmware's entire task stack. The caller decides where the
/// storage lives (a `static`, or a leaked heap slice as the firmware does), and
/// it must not be empty.
///
/// Samples are pushed at a fixed rate, so the timestamp of any element is
/// `get(i).t_ms`; ordering is oldest-first by index 0..len.
pub struct SampleRing<'a> {
    buf: &'a mut [Sample],
    /// Next write position.
    write: usize,
    /// Number of valid samples (<= capacity).
    len: usize,
    /// Total samples ever pushed (for detecting drops/overrun).
    pushed: u64,
}

impl<'a> SampleRing<'a> {
    pub fn new(buf: &'a mut [Sample]) -> Self {
        debug_assert!(!buf.is_empty(), "a sample ring needs a non-empty buffer");
        SampleRing {
            buf,
            write: 0,
            len: 0,
            pushed: 0,
        }
    }

    /// Push a sample, evicting the oldest if full.
    pub fn push(&mut self, s: Sample) {
        let n = self.buf.len();
        self.buf[self.write] = s;
        self.write = (self.write + 1) % n;
        self.len = (self.len + 1).min(n);
        self.pushed += 1;
    }

    /// Number of live samples.
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    /// Total samples ever pushed.
    pub fn pushed(&self) -> u64 {
        self.pushed
    }

    /// Get the `i`-th oldest live sample (`0` = oldest).
    pub fn get(&self, i: usize) -> Option<&Sample> {
        if i >= self.len {
            return None;
        }
        let n = self.buf.len();
        // Oldest live sample sits at `write` once the ring is full; before
        // that it sits at 0.
        let oldest = if self.len < n { 0 } else { self.write };
        Some(&self.buf[(oldest + i) % n])
    }

    /// Most recent sample.
    pub fn latest(&self) -> Option<&Sample> {
        self.get(self.len.checked_sub(1)?)
    }

    /// Index of the first live sample at or after `t_ms`.
    pub fn index_at_or_after(&self, t_ms: u32) -> Option<usize> {
        for i in 0..self.len {
            if self.get(i)?.t_ms >= t_ms {
                return Some(i);
            }
        }
        None
    }

    /// Copy the samples in `[from_ms - before_ms, to_ms + after_ms]` into
    /// `out`, oldest first. Returns how many were copied (clipped to
    /// `out.len()` and to the live window).
    ///
    /// This is how a detected rep's window is handed to `rep::compute_rep`,
    /// which needs a contiguous slice (the ring isn't contiguous once it has
    /// wrapped).
    pub fn copy_window(
        &self,
        from_ms: u32,
        to_ms: u32,
        before_ms: u32,
        after_ms: u32,
        out: &mut [Sample],
    ) -> usize {
        let start_ms = from_ms.saturating_sub(before_ms);
        let end_ms = to_ms.saturating_add(after_ms);
        let Some(mut i) = self.index_at_or_after(start_ms) else {
            return 0;
        };
        let mut n = 0;
        while n < out.len() && i < self.len {
            let Some(s) = self.get(i) else { break };
            if s.t_ms > end_ms {
                break;
            }
            out[n] = *s;
            n += 1;
            i += 1;
        }
        n
    }

    /// Iterate live samples, oldest first.
    pub fn for_each(&self, mut f: impl FnMut(usize, &Sample)) {
        for i in 0..self.len {
            if let Some(s) = self.get(i) {
                f(i, s);
            }
        }
    }
}
