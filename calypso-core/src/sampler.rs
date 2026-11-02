//! 100 Hz IMU sampling: fixed-point sample format and a RAM ring buffer.

/// Specific-force scale: LSB per m/s² (the BNO055's accelerometer registers, so no conversion is needed on the device).
pub const ACCEL_SCALE: i32 = 100;

/// Angular-rate scale: LSB per degree per second (the BNO055's gyroscope registers).
pub const GYRO_SCALE: i32 = 16;

/// Quaternion scale: LSB per unit quaternion component, for the attitude computed here rather than measured.
pub const QUAT_SCALE: i32 = 1 << 14;

pub const SAMPLE_RATE_HZ: u32 = 100;

/// What the sensor was set to when a rep was recorded: what the numbers in a stored trace mean, and
/// which detection logic closed the rep.
///
/// It travels in a rep's summary rather than being assumed, because a trace is read on a different
/// day by a different build — and because the scales here are the ones the chip's registers use,
/// which its range setting does *not* change. A field left at 0 is one the device did not record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RepConfig {
    /// The accelerometer's configured clip point, in g.
    pub accel_range_g: u32,
    /// The gyroscope's configured clip point, in degrees per second.
    pub gyro_range_dps: u32,
    /// The accelerometer's configured bandwidth, in hertz.
    pub accel_bandwidth_hz: u32,
    /// The gyroscope's configured bandwidth, in hertz.
    pub gyro_bandwidth_hz: u32,
    pub sample_rate_hz: u32,
    pub detector_version: u32,
}

impl RepConfig {
    /// What the sampler itself knows: the rate it samples at and the version of the logic that
    /// closed the rep. The sensor's own settings are the driver's to fill in.
    pub const fn of(rate_hz: u32, detector_version: u32) -> Self {
        RepConfig {
            sample_rate_hz: rate_hz,
            detector_version,
            ..RepConfig::EMPTY
        }
    }

    /// Nothing recorded, which is what a device that has not configured its sensor reports.
    pub const EMPTY: RepConfig = RepConfig {
        accel_range_g: 0,
        gyro_range_dps: 0,
        accel_bandwidth_hz: 0,
        gyro_bandwidth_hz: 0,
        sample_rate_hz: 0,
        detector_version: 0,
    };
}

/// One IMU sample, fixed point.
///
/// - `t_ms`: monotonic timestamp in milliseconds.
/// - `accel`: **specific force**, `ACCEL_SCALE` LSB/m/s² — gravity still in it, so a still sensor
///   reads 1 g up its own up axis. This is the raw accelerometer, not an acceleration.
/// - `gyro`: angular rate, `GYRO_SCALE` LSB/dps, in the sensor's own frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Sample {
    pub t_ms: u32,
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
}

impl Sample {
    pub const ZERO: Sample = Sample {
        t_ms: 0,
        accel: [0; 3],
        gyro: [0; 3],
    };

    /// Build from floating-point readings (m/s² of specific force, degrees per second).
    pub fn from_f32(t_ms: u32, specific_force: [f32; 3], gyro_dps: [f32; 3]) -> Self {
        Sample {
            t_ms,
            accel: specific_force.map(|a| quantize(a, ACCEL_SCALE)),
            gyro: gyro_dps.map(|g| quantize(g, GYRO_SCALE)),
        }
    }

    /// The specific force in m/s², gravity included.
    pub fn specific_force_f32(&self) -> [f32; 3] {
        self.accel.map(|a| a as f32 / ACCEL_SCALE as f32)
    }

    /// Angular rate in degrees per second.
    pub fn gyro_f32(&self) -> [f32; 3] {
        self.gyro.map(|g| g as f32 / GYRO_SCALE as f32)
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
/// Borrows its buffer rather than owning it, like [`crate::rep::TraceBuf`]: a 30 s window as an owned value is 54 KB on the stack before it can move to the heap, and the caller has to decide whether it is a `static` or leaked.
/// The buffer must not be empty.
/// Samples are pushed at a fixed rate, so the timestamp of any element is `get(i).t_ms`.
pub struct SampleRing<'a> {
    buf: &'a mut [Sample],
    write: usize,
    len: usize,
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

    pub fn push(&mut self, s: Sample) {
        let n = self.buf.len();
        self.buf[self.write] = s;
        self.write = (self.write + 1) % n;
        self.len = (self.len + 1).min(n);
        self.pushed += 1;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    pub fn pushed(&self) -> u64 {
        self.pushed
    }

    /// Get the `i`-th oldest live sample (`0` = oldest).
    pub fn get(&self, i: usize) -> Option<&Sample> {
        if i >= self.len {
            return None;
        }
        let n = self.buf.len();
        // Oldest live sample sits at `write` once the ring is full; before that it sits at 0.
        let oldest = if self.len < n { 0 } else { self.write };
        Some(&self.buf[(oldest + i) % n])
    }

    pub fn latest(&self) -> Option<&Sample> {
        self.get(self.len.checked_sub(1)?)
    }

    pub fn index_at_or_after(&self, t_ms: u32) -> Option<usize> {
        for i in 0..self.len {
            if self.get(i)?.t_ms >= t_ms {
                return Some(i);
            }
        }
        None
    }

    /// Copy the samples in `[from_ms - before_ms, to_ms + after_ms]` into `out`, oldest first, returning how many were copied — clipped to `out.len()` and to the live window.
    /// The ring is not contiguous once it has wrapped.
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

    pub fn for_each(&self, mut f: impl FnMut(usize, &Sample)) {
        for i in 0..self.len {
            if let Some(s) = self.get(i) {
                f(i, s);
            }
        }
    }
}
