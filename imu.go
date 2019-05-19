package filter

// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// golang code adaptation for this two greatest IMU Libraries for Arduino
// https://github.com/PaulStoffregen/MadgwickAHRS
// https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

import "math"

var (
	beta     = 0.1
	g        = 9.81
	radtodeg = 57.29578
)

var (
	sampleFreqDef float64
	invSampleFreq float64
	axx           float64
	ayy           float64
	azz           float64
	ax            float64
	ay            float64
	az            float64
	gx            float64
	gy            float64
	gz            float64
)

// Quaternion is a four-element vector that can be used to encode any rotation in a 3D coordinate system
type Quaternion struct {
	q0 float64
	q1 float64
	q2 float64
	q3 float64
}

// Begin - initialize quaternion and sample rate
func (v *Quaternion) Begin(s float64) {

	sampleFreqDef = s
	invSampleFreq = 1.0 / sampleFreqDef
	v.q0 = 1.0
	v.q1 = 0.0
	v.q2 = 0.0
	v.q3 = 0.0
}

// for normalize vectors
func invSqrt(x float64) float64 {
	return 1.0 / math.Sqrt(x)
}

// for conjugate a vector
func (v Quaternion) conj() Quaternion {
	c := Quaternion{v.q0, -v.q1, -v.q2, -v.q3}
	return c
}

// for quaternion multiplication
func (v Quaternion) mult(c Quaternion) Quaternion {
	var m Quaternion

	// is defined by:
	// (Q1 * Q2).q0 = (Q1.q0*Q2.q0 - Q1.q1*Q2.q1 - Q1.q2*Q2.q2 - Q1.q3*Q2.q3)
	// (Q1 * Q2).q1 = (Q1.q0*Q2.q1 + Q1.q1*Q2.q0 + Q1.q2*Q2.q3 - Q1.q3*Q2.q2)
	// (Q1 * Q2).q2 = (Q1.q0*Q2.q2 - Q1.q1*Q2.q3 + Q1.q2*Q2.q0 + Q1.q3*Q2.q1)
	// (Q1 * Q2).q3 = (Q1.q0*Q2.q3 + Q1.q1*Q2.q2 - Q1.q2*Q2.q1 + Q1.q3*Q2.q0)

	m.q0 = v.q0*c.q0 - v.q1*c.q1 - v.q2*c.q2 - v.q3*c.q3
	m.q1 = v.q0*c.q1 + v.q1*c.q0 + v.q2*c.q3 - v.q3*c.q2
	m.q2 = v.q0*c.q2 - v.q1*c.q3 + v.q2*c.q0 + v.q3*c.q1
	m.q3 = v.q0*c.q3 + v.q1*c.q2 - v.q2*c.q1 + v.q3*c.q0

	return m
}

// gyroscope - radians/sec

// UpdateIMU - update data from sensors
func (v *Quaternion) UpdateIMU(gyrox, gyroy, gyroz, acclx, accly, acclz float64) {

	axx = acclx
	ayy = accly
	azz = acclz
	ax = acclx
	ay = accly
	az = acclz
	gx = gyrox
	gy = gyroy
	gz = gyroz

	var recipNorm float64
	var s0, s1, s2, s3 float64
	var qDot1, qDot2, qDot3, qDot4 float64
	var _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3 float64

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-v.q1*gx - v.q2*gy - v.q3*gz)
	qDot2 = 0.5 * (v.q0*gx + v.q2*gz - v.q3*gy)
	qDot3 = 0.5 * (v.q0*gy - v.q1*gz + v.q3*gx)
	qDot4 = 0.5 * (v.q0*gz + v.q1*gy - v.q2*gx)

	if !((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax*ax + ay*ay + az*az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0 * v.q0
		_2q1 = 2.0 * v.q1
		_2q2 = 2.0 * v.q2
		_2q3 = 2.0 * v.q3
		_4q0 = 4.0 * v.q0
		_4q1 = 4.0 * v.q1
		_4q2 = 4.0 * v.q2
		_8q1 = 8.0 * v.q1
		_8q2 = 8.0 * v.q2
		q0q0 = v.q0 * v.q0
		q1q1 = v.q1 * v.q1
		q2q2 = v.q2 * v.q2
		q3q3 = v.q3 * v.q3

		// Gradient decent algorithm corrective step
		s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
		s1 = _4q1*q3q3 - _2q3*ax + 4.0*q0q0*v.q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
		s2 = 4.0*q0q0*v.q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
		s3 = 4.0*q1q1*v.q3 - _2q1*ax + 4.0*q2q2*v.q3 - _2q2*ay

		recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3) // normalise step magnitude
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		// Apply feedback step
		qDot1 -= beta * s0
		qDot2 -= beta * s1
		qDot3 -= beta * s2
		qDot4 -= beta * s3
	}
	v.q0 += qDot1 * invSampleFreq
	v.q1 += qDot2 * invSampleFreq
	v.q2 += qDot3 * invSampleFreq
	v.q3 += qDot4 * invSampleFreq

	// Normalise quaternion
	recipNorm = invSqrt(v.q0*v.q0 + v.q1*v.q1 + v.q2*v.q2 + v.q3*v.q3)
	v.q0 *= recipNorm
	v.q1 *= recipNorm
	v.q2 *= recipNorm
	v.q3 *= recipNorm

}

// GetRoll - get the euler angles (roll)
func (v Quaternion) GetRoll() float64 {
	return math.Atan2(v.q0*v.q1+v.q2*v.q3, 0.5-v.q1*v.q1-v.q2*v.q2) * radtodeg
}

// GetPitch - get the euler angles (pitch)
func (v Quaternion) GetPitch() float64 {
	return math.Asin(2.0*(v.q0*v.q2-v.q1*v.q3)) * radtodeg
}

// GetYaw - get the euler angles (yaw)
func (v Quaternion) GetYaw() float64 {
	return math.Atan2(v.q1*v.q2+v.q0*v.q3, 0.5-v.q2*v.q2-v.q3*v.q3) * radtodeg
}

// GetLinearAccel - get the acceleration without gravity
func (v Quaternion) GetLinearAccel() [3]float64 {

	// get the linear accel
	// subtracting gravity from accel for each axis
	var Linear [3]float64

	/* the rotation matrix is defined by:
	v.q0*v.q0 + v.q1*v.q1 - v.q2*v.q2 - v.q3*v.q3 ; 2*v.q1*v.q2 - 2*v.q0*v.q3 ; 2*v.q1*v.q3 + 2*v.q0*v.q2
	2*v.q1*v.q2 + 2*v.q0*v.q3 ; v.q0*v.q0 - v.q1*v.q1 + v.q2*v.q2 - v.q3*v.q3 ; 2*v.q2*v.q3 - 2*v.q0*v.q1
	2*v.q1*v.q3 - 2*v.q0*v.q2 ; 2*v.q2*v.q3 + 2*v.q0*v.q1 ; v.q0*v.q0 - v.q1*v.q1 - v.q2*v.q2 + v.q3*v.q3
	*/

	// (gravity only have z component)
	Linear[0] = axx - (2*v.q1*v.q3-2*v.q0*v.q2)*g
	Linear[1] = ayy - (2*v.q2*v.q3+2*v.q0*v.q1)*g
	Linear[2] = azz - (v.q0*v.q0-v.q1*v.q1-v.q2*v.q2+v.q3*v.q3)*g

	return Linear
}

// GetWorldAccel - get usefull data from accel and orientation
func (v Quaternion) GetWorldAccel() [3]float64 {

	Linear := v.GetLinearAccel()

	// get the real world accel (without rotation)
	// P_out = Q * P_in * conj(Q)	(P_in and P_Out are points in 3D space)
	// conj(Q) is the conjugate of the orientation quaternion (Q=[q0,q1,q2,q3], q'=[q0,-q1,-q2,-q3])
	var World [3]float64

	// create a new quaternion
	p := Quaternion{0, Linear[0], Linear[1], Linear[2]}

	// quaternion multiplication: v * p, stored back in p
	p = v.mult(p)

	// quaternion multiplication: p * conj(v), stored back in p
	p = p.mult(v.conj())

	// only need x y z from the Quaternion {w, x, y, z}
	World[0] = p.q1
	World[1] = p.q2
	World[2] = p.q3

	return World
}
