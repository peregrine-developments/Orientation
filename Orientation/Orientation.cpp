#include "Orientation.h"
#include "Quaternion.h"
#include "Arduino.h"

Quaternion expectedGravity(1, 0, 0);

EulerAngles Orientation::quaternionToEuler(Quaternion q)
{
    EulerAngles ret;

    float sinr_cosp = 2 * (q.a * q.b + q.c * q.d);
    float cosr_cosp = 1 - 2 * (q.b * q.b + q.c * q.c);
    ret.roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (q.a * q.c + -q.d * q.b);
    if (abs(sinp) >= 1)
        ret.pitch = copysign(PI / 2, sinp); // return 90 if out of range
    else
        ret.pitch = asin(sinp);

    float siny_cosp = 2 * (q.a * q.d + q.b * q.c);
    float cosy_cosp = 1 - 2 * (q.c * q.c + q.d * q.d);
    ret.yaw = atan2(siny_cosp, cosy_cosp);

    return ret;
}

void Orientation::update(float yaw, float pitch, float roll, float dt)
{
    float norm = sqrtf(powf(yaw, 2) + powf(pitch, 2) + powf(roll, 2));
    norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0

    orientation *= Quaternion::from_axis_angle(dt * norm, roll / norm, pitch / norm, yaw / norm);
}
void Orientation::update(EulerAngles gyroMeasure, float dt)
{
    float norm = sqrtf(powf(gyroMeasure.yaw, 2) + powf(gyroMeasure.pitch, 2) + powf(gyroMeasure.roll, 2));
    norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0

    orientation *= Quaternion::from_axis_angle(dt * norm, gyroMeasure.roll / norm, gyroMeasure.pitch / norm, gyroMeasure.yaw / norm);
}

void Orientation::updateOld(float yaw, float pitch, float roll, float dt)
{
    orientation *= Quaternion::from_euler_rotation(yaw * dt, pitch * dt, roll * dt);
}
void Orientation::updateOld(EulerAngles gyroMeasure, float dt)
{
    orientation *= Quaternion::from_euler_rotation(gyroMeasure.yaw * dt, gyroMeasure.pitch * dt, gyroMeasure.roll * dt);
}

void Orientation::updateApprox(float yaw, float pitch, float roll, float dt)
{
    orientation *= Quaternion::from_euler_rotation_approx(yaw * dt, pitch * dt, roll * dt);
}
void Orientation::updateApprox(EulerAngles gyroMeasure, float dt)
{
    orientation *= Quaternion::from_euler_rotation_approx(gyroMeasure.yaw * dt, gyroMeasure.pitch * dt, gyroMeasure.roll * dt);
}

void Orientation::updateGravity(float x, float y, float z)
{
    Quaternion measuredGravity = Quaternion(x, y, z);
    worldGravity = orientation.rotate(measuredGravity);
}

void Orientation::applyComplementary(Quaternion gravity, float alpha)
{
    Quaternion correction = Quaternion(gravity).normalize().rotation_between_vectors(expectedGravity);
    correction = orientation.conj().rotate(correction.fractional(alpha)); // may just be orientation.rotate(correction.fractional(alpha)) idk
    orientation *= correction.normalize();
}

void Orientation::zeroRoll()
{
    EulerAngles angleRep = this->toEuler();
    orientation = Quaternion::from_euler_rotation(angleRep.yaw, angleRep.pitch, 0);
}