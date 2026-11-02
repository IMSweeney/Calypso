import 'package:calypso/model/calibration.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/state/calibration.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('the magnetometer is reported but never asked for', () {
    final onlyMagShort = pb.CalibrationStatus(
      sys: maxCalibrationLevel,
      accel: maxCalibrationLevel,
      gyro: maxCalibrationLevel,
      mag: 0,
      ready: false,
    );

    expect(
      onlyMagShort.levels,
      contains('mag 0/3'),
      reason: 'it is still read: "read each sensor" is the point',
    );
    expect(onlyMagShort.action.toLowerCase(), isNot(contains('mag')));
    expect(onlyMagShort.action.toLowerCase(), isNot(contains('figure')));
  });

  test('the action names only the sensors that are short', () {
    final bothShort = pb.CalibrationStatus(
      sys: 1,
      accel: 1,
      gyro: 2,
      mag: 0,
      ready: false,
    );
    final gyroOnly = pb.CalibrationStatus(
      sys: 3,
      accel: 3,
      gyro: 2,
      mag: 0,
      ready: false,
    );
    final accelOnly = pb.CalibrationStatus(
      sys: 3,
      accel: 2,
      gyro: 3,
      mag: 0,
      ready: false,
    );

    expect(gyroOnly.action, contains('still on the rack'));
    expect(gyroOnly.action, isNot(contains('upright')));

    expect(accelOnly.action, contains('upright'));
    expect(accelOnly.action, isNot(contains('on the rack')));

    expect(bothShort.action, contains('on the rack'));
    expect(bothShort.action, contains('upright'));
  });

  test('an accelerometer that has wandered is a hint, not a pause', () {
    // The device's gate is the gyroscope alone, so this device is recording reps — with `ready` true — while its
    // accelerometer reads short. That is the case the hint has to cover and, before this, the one it never showed.
    final accelShort = pb.CalibrationStatus(
      sys: 3,
      accel: 1,
      gyro: 3,
      mag: 0,
      ready: true,
    );
    expect(accelShort.records, isTrue);
    expect(accelShort.needsAttention, isTrue);
    expect(accelShort.summary, isNot(contains('paused')));
    expect(accelShort.summary, contains('travel'));
    expect(accelShort.action, contains('upright'));
    expect(accelShort.action, isNot(contains('on the rack')));
    expect(accelShort.action, contains('still record'));

    final gyroShort = pb.CalibrationStatus(
      sys: 3,
      accel: 3,
      gyro: 2,
      mag: 0,
      ready: false,
    );
    expect(gyroShort.records, isFalse);
    expect(gyroShort.needsAttention, isTrue);
    expect(gyroShort.summary, contains('paused'));

    final fully = pb.CalibrationStatus(
      sys: 3,
      accel: 3,
      gyro: 3,
      mag: 0,
      ready: true,
    );
    expect(fully.needsAttention, isFalse, reason: 'nothing to say');
  });

  test(
    'nothing is asked for when the sensors are fine but the device is not recording',
    () {
      final finishing = pb.CalibrationStatus(
        sys: 2,
        accel: 3,
        gyro: 3,
        mag: 0,
        ready: false,
      );
      final gyroOnly = pb.CalibrationStatus(
        sys: 3,
        accel: 3,
        gyro: 2,
        mag: 0,
        ready: false,
      );

      expect(finishing.action, isNotEmpty);
      expect(finishing.action, isNot(gyroOnly.action));
    },
  );

  group('CalibrationNotifier', () {
    test('is not ready before the device has said anything', () {
      expect(CalibrationNotifier().isReady, isFalse);
      expect(CalibrationNotifier().latest, isNull);
    });

    test('notifies on a change, and stays quiet on a repeat', () {
      final notifier = CalibrationNotifier();
      var notifications = 0;
      notifier.addListener(() => notifications++);

      final uncalibrated = pb.CalibrationStatus(
        sys: 1,
        accel: 1,
        gyro: 2,
        mag: 0,
        ready: false,
      );
      notifier.set(uncalibrated);
      expect(notifications, 1);

      notifier.set(uncalibrated);
      expect(notifications, 1, reason: 'same reading, nothing to say');

      notifier.set(
        pb.CalibrationStatus(
          sys: maxCalibrationLevel,
          accel: maxCalibrationLevel,
          gyro: maxCalibrationLevel,
          mag: 0,
          ready: true,
        ),
      );
      expect(notifications, 2);
      expect(notifier.isReady, isTrue);
    });

    test('clearing forgets the reading, and an empty clear is a no-op', () {
      final notifier = CalibrationNotifier();
      var notifications = 0;
      notifier.addListener(() => notifications++);

      notifier.clear();
      expect(notifications, 0, reason: 'nothing to forget');

      notifier.set(
        pb.CalibrationStatus(sys: 1, accel: 1, gyro: 1, mag: 0, ready: false),
      );
      expect(notifications, 1);

      notifier.clear();
      expect(notifications, 2);
      expect(notifier.latest, isNull);
      expect(notifier.isReady, isFalse);
    });
  });
}
