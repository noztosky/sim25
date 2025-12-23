#pragma once

#include <cmath>

namespace AttitudeUtils {
	inline void quatInverse(float qw, float qx, float qy, float qz,
		float& rw, float& rx, float& ry, float& rz)
	{
		// for unit quaternion, inverse = conjugate
		rw = qw; rx = -qx; ry = -qy; rz = -qz;
	}

	inline void quatMultiply(
		float aw, float ax, float ay, float az,
		float bw, float bx, float by, float bz,
		float& rw, float& rx, float& ry, float& rz)
	{
		rw = aw*bw - ax*bx - ay*by - az*bz;
		rx = aw*bx + ax*bw + ay*bz - az*by;
		ry = aw*by - ax*bz + ay*bw + az*bx;
		rz = aw*bz + ax*by - ay*bx + az*bw;
	}

	inline float computeYawRad(float qw, float qx, float qy, float qz)
	{
		float siny_cosp = 2.0f * (qw * qz + qx * qy);
		float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
		return std::atan2(siny_cosp, cosy_cosp);
	}

	inline void makeYawQuat(float yaw_rad, float& qw, float& qx, float& qy, float& qz)
	{
		float cy = std::cos(yaw_rad * 0.5f);
		float sy = std::sin(yaw_rad * 0.5f);
		qw = cy; qx = 0.0f; qy = 0.0f; qz = sy;
	}

	inline void computeSmallAngleErrorExEy(
		float qd_w, float qd_x, float qd_y, float qd_z,
		float qe_w, float qe_x, float qe_y, float qe_z,
		float& ex, float& ey)
	{
		// q_err = q_des * inverse(q_est) = q_des * conj(q_est)
		float be_w, be_x, be_y, be_z;
		quatInverse(qe_w, qe_x, qe_y, qe_z, be_w, be_x, be_y, be_z);
		float er_w, er_x, er_y, er_z;
		quatMultiply(qd_w, qd_x, qd_y, qd_z, be_w, be_x, be_y, be_z, er_w, er_x, er_y, er_z);
		if (er_w < 0) { er_w = -er_w; er_x = -er_x; er_y = -er_y; er_z = -er_z; }
		ex = 2.0f * er_x; ey = 2.0f * er_y;
	}
	inline void computeEulerDeg(float qw, float qx, float qy, float qz,
		float& roll_deg, float& pitch_deg, float& yaw_deg)
	{
		// Roll (x-axis rotation)
		float sinr_cosp = 2.0f * (qw * qx + qy * qz);
		float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
		float roll = std::atan2(sinr_cosp, cosr_cosp);

		// Pitch (y-axis rotation)
		float sinp = 2.0f * (qw * qy - qz * qx);
		if (sinp > 1.0f) sinp = 1.0f; else if (sinp < -1.0f) sinp = -1.0f;
		float pitch = std::asin(sinp);

		// Yaw (z-axis rotation)
		float siny_cosp = 2.0f * (qw * qz + qx * qy);
		float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
		float yaw = std::atan2(siny_cosp, cosy_cosp);

		const float RAD2DEG = 57.29578f;
		roll_deg = roll * RAD2DEG;
		pitch_deg = pitch * RAD2DEG;
		yaw_deg = yaw * RAD2DEG;
	}
}


