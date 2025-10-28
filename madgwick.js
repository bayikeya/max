// MadgwickAHRS - lightweight JS implementation
// Usage:
// const ahrs = new Madgwick(0.1);
// ahrs.update(gyroX, gyroY, gyroZ, accX, accY, accZ, magX, magY, magZ);
// const euler = ahrs.getEuler(); // {yaw, pitch, roll} in degrees

class Madgwick {
  constructor(beta=0.1, sampleFreq=50) {
    this.beta = beta;
    this.sampleFreq = sampleFreq;
    this.q0 = 1; this.q1 = 0; this.q2 = 0; this.q3 = 0;
  }
  invSqrt(x){ return 1/Math.sqrt(x); }
  // gyro in deg/s, acc in m/s^2, mag optional
  update(gx,gy,gz, ax,ay,az, mx=null,my=null,mz=null) {
    // convert gyro to rad/s
    const deg2rad = Math.PI/180;
    gx *= deg2rad; gy *= deg2rad; gz *= deg2rad;
    let q0=this.q0,q1=this.q1,q2=this.q2,q3=this.q3;
    // Normalise accelerometer
    let norm = Math.sqrt(ax*ax + ay*ay + az*az);
    if (norm === 0) return;
    ax/=norm; ay/=norm; az/=norm;
    // Madgwick algorithm (gyroscope + accelerometer only variant)
    // Reference: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
    const _2q0 = 2*q0, _2q1=2*q1, _2q2=2*q2, _2q3=2*q3;
    const _4q0 = 4*q0, _4q1=4*q1, _4q2=4*q2;
    const _8q1 = 8*q1, _8q2=8*q2;
    const q0q0 = q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;
    // Gradient decent algorithm corrective step
    const s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    const s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    const s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    const s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay;
    norm = Math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm === 0) return;
    const recipNorm = 1 / norm;
    // apply feedback step
    q0 -= (this.beta * s0 * recipNorm);
    q1 -= (this.beta * s1 * recipNorm);
    q2 -= (this.beta * s2 * recipNorm);
    q3 -= (this.beta * s3 * recipNorm);
    // integrate rate of change
    const gx2 = 0.5 * (-q1*gx - q2*gy - q3*gz);
    const gy2 = 0.5 * ( q0*gx + q2*gz - q3*gy);
    const gz2 = 0.5 * ( q0*gy - q1*gz + q3*gx);
    q0 += gx2 / this.sampleFreq;
    q1 += gy2 / this.sampleFreq;
    q2 += gz2 / this.sampleFreq;
    // normalise quaternion
    norm = Math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0/=norm; q1/=norm; q2/=norm; q3/=norm;
    this.q0=q0; this.q1=q1; this.q2=q2; this.q3=q3;
  }
  getEuler() {
    const q0=this.q0,q1=this.q1,q2=this.q2,q3=this.q3;
    const yaw = Math.atan2(2*(q1*q2+q0*q3), q0*q0+q1*q1-q2*q2-q3*q3) * 180/Math.PI;
    const pitch = Math.asin(Math.max(-1, Math.min(1, 2*(q0*q2 - q1*q3)))) * 180/Math.PI;
    const roll = Math.atan2(2*(q0*q1+q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 180/Math.PI;
    // Normalize yaw to 0-360
    let yaw360 = (yaw + 360) % 360;
    return { yaw: yaw360, pitch, roll };
  }
}

// export for browser
window.Madgwick = Madgwick;
