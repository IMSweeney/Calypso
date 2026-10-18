// The calibration model: what arrives from the device, and the wording derived
// from it. No device and no widgets — this is the part that decides what the user
// is told to do, so it is worth pinning down on its own.

import 'package:calypso/model/calibration.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/state/calibration.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('the wire form is mapped level by level', () {
    final status = CalibrationStatus.fromProto(
      pb.CalibrationStatus(sys: 3, accel: 2, gyro: 1, mag: 0, ready: false),
    );

    expect(status.sys, 3);
    expect(status.accel, 2);
    expect(status.gyro, 1);
    expect(status.mag, 0);
    expect(status.ready, isFalse);
  });

  test('ready is the device\'s answer, not one re-derived here', () {
    // The device sends the gate it is acting on. If the app re-derived it, a
    // firmware that changed the rule (dropping the aggregate, say) would leave
    // the app confidently telling the user the opposite of what is happening.
    final saysReady = CalibrationStatus.fromProto(
      pb.CalibrationStatus(sys: 3, accel: 3, gyro: 3, mag: 0, ready: true),
    );
    expect(saysReady.ready, isTrue);

    final saysNotReady = CalibrationStatus.fromProto(
      pb.CalibrationStatus(sys: 1, accel: 1, gyro: 1, mag: 0, ready: false),
    );
    expect(saysNotReady.ready, isFalse);
  });

  test('the magnetometer is reported but never asked for', () {
    // The trap this hardware sets: the BNO055 runs in IMU mode, where the
    // magnetometer never calibrates. A hint that asked the user to fix `mag`
    // would send them to do figure eights in the gym forever.
    final onlyMagShort = CalibrationStatus(
      sys: maxCalibrationLevel,
      accel: maxCalibrationLevel,
      gyro: maxCalibrationLevel,
      mag: 0,
      ready: false,
    );

    expect(onlyMagShort.levels, contains('mag 0/3'),
        reason: 'it is still read: "read each sensor" is the point');
    expect(onlyMagShort.action.toLowerCase(), isNot(contains('mag')));
    expect(onlyMagShort.action.toLowerCase(), isNot(contains('figure')));
  });

  test('the action names only the sensors that are short', () {
    final bothShort = CalibrationStatus(sys: 1, accel: 1, gyro: 2, mag: 0, ready: false);
    final gyroOnly = CalibrationStatus(sys: 3, accel: 3, gyro: 2, mag: 0, ready: false);
    final accelOnly = CalibrationStatus(sys: 3, accel: 2, gyro: 3, mag: 0, ready: false);

    // Stillness is what the gyroscope wants; a few still orientations is what the
    // accelerometer wants. Each hint asks for one, not for both.
    expect(gyroOnly.action, contains('still on the rack'));
    expect(gyroOnly.action, isNot(contains('upright')));

    expect(accelOnly.action, contains('upright'));
    expect(accelOnly.action, isNot(contains('on the rack')));

    expect(bothShort.action, contains('on the rack'));
    expect(bothShort.action, contains('upright'));
  });

  test('nothing is asked for when the sensors are fine but the device is not recording', () {
    // Both levels good, ready still false: the aggregate is catching up. There is
    // no action to give, and inventing one would send the user off to no effect.
    final finishing = CalibrationStatus(sys: 2, accel: 3, gyro: 3, mag: 0, ready: false);
    final gyroOnly = CalibrationStatus(sys: 3, accel: 3, gyro: 2, mag: 0, ready: false);

    expect(finishing.action, isNotEmpty);
    expect(finishing.action, isNot(gyroOnly.action));
  });

  test('the level line carries every sensor', () {
    const status = CalibrationStatus(sys: 1, accel: 2, gyro: 3, mag: 0, ready: false);
    expect(status.levels, 'accel 2/3 · gyro 3/3 · sys 1/3 · mag 0/3');
  });

  group('CalibrationNotifier', () {
    test('is not ready before the device has said anything', () {
      // The safe thing to believe: a device we have heard nothing from is not
      // recording.
      expect(CalibrationNotifier().isReady, isFalse);
      expect(CalibrationNotifier().latest, isNull);
    });

    test('notifies on a change, and stays quiet on a repeat', () {
      // The device pushes once a second and the levels rarely move: rebuilding
      // the list screen every second for an identical reading is the thing this
      // guards against.
      final notifier = CalibrationNotifier();
      var notifications = 0;
      notifier.addListener(() => notifications++);

      const uncalibrated = CalibrationStatus(sys: 1, accel: 1, gyro: 2, mag: 0, ready: false);
      notifier.set(uncalibrated);
      expect(notifications, 1);

      notifier.set(uncalibrated);
      expect(notifications, 1, reason: 'same reading, nothing to say');

      notifier.set(const CalibrationStatus(
        sys: maxCalibrationLevel,
        accel: maxCalibrationLevel,
        gyro: maxCalibrationLevel,
        mag: 0,
        ready: true,
      ));
      expect(notifications, 2);
      expect(notifier.isReady, isTrue);
    });

    test('clearing forgets the reading, and an empty clear is a no-op', () {
      final notifier = CalibrationNotifier();
      var notifications = 0;
      notifier.addListener(() => notifications++);

      notifier.clear();
      expect(notifications, 0, reason: 'nothing to forget');

      notifier.set(const CalibrationStatus(sys: 1, accel: 1, gyro: 1, mag: 0, ready: false));
      expect(notifications, 1);

      notifier.clear();
      expect(notifications, 2);
      expect(notifier.latest, isNull);
      expect(notifier.isReady, isFalse);
    });
  });
}
