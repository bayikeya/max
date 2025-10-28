// Simple EKF for 2D position with heading bias and step-scale factor.
// State vector: [x, y, theta_bias, scale]
// Units: x,y in meters (local ENU), theta_bias in radians, scale dimensionless
// Process model: when a step occurs, control u = step_length (m) and measured heading theta_m (rad).
// x,y updated: x += u * scale * cos(theta_m + theta_bias), y += u*scale*sin(...)
// GPS observation: z = [x, y] (meters) with covariance from GPS accuracy.

function zeros(n,m){ const A=[]; for(let i=0;i<n;i++){ A.push(Array(m).fill(0)); } return A; }
function identity(n){ const I=zeros(n,n); for(let i=0;i<n;i++) I[i][i]=1; return I; }
function matAdd(A,B){ return A.map((r,i)=> r.map((v,j)=> v + B[i][j])); }
function matSub(A,B){ return A.map((r,i)=> r.map((v,j)=> v - B[i][j])); }
function matMul(A,B){
  const m=A.length, p=B[0].length, n=A[0].length;
  const C=zeros(m,p);
  for(let i=0;i<m;i++) for(let k=0;k<n;k++) if(A[i][k]!==0) for(let j=0;j<p;j++) C[i][j]+=A[i][k]*B[k][j];
  return C;
}
function transpose(A){ return A[0].map((_,i)=> A.map(r=> r[i])); }

class SimpleEKF {
  constructor() {
    // state and covariance
    this.x = [0,0,0,1.0]; // x,y,theta_bias (rad), scale
    this.P = identity(4).map(r=> r.map(v=> v*0.5));
    // process noise Q small
    this.Q = identity(4).map(r=> r.map(v=> v*0.01));
    // origin (lat,lon) to convert meters
    this.origin = null;
  }
  setOrigin(lat, lon) {
    this.origin = { lat, lon };
  }
  // convert lat/lon to meters ENU approximate (east, north) relative to origin
  ll_to_xy(lat, lon) {
    if (!this.origin) throw new Error('origin not set');
    const dLat = (lat - this.origin.lat) * Math.PI/180;
    const dLon = (lon - this.origin.lon) * Math.PI/180;
    const R = 6378137;
    const x = dLon * R * Math.cos(this.origin.lat * Math.PI/180); // east
    const y = dLat * R; // north
    return [x,y];
  }
  xy_to_ll(x,y) {
    const R = 6378137;
    const dLat = y / R;
    const dLon = x / (R * Math.cos(this.origin.lat * Math.PI/180));
    const lat = this.origin.lat + dLat * 180/Math.PI;
    const lon = this.origin.lon + dLon * 180/Math.PI;
    return [lat,lon];
  }

  // Predict step via control: stepLen (m), theta_meas (rad)
  predict_step(stepLen, theta_meas) {
    const [x,y,theta_bias,scale] = this.x;
    const theta = theta_meas + theta_bias;
    const dx = stepLen * scale * Math.cos(theta);
    const dy = stepLen * scale * Math.sin(theta);
    // State update
    this.x[0] = x + dx;
    this.x[1] = y + dy;
    // Jacobian F (state transition wrt state)
    // x' = x + stepLen*scale*cos(theta_meas + theta_bias)
    // ∂x'/∂theta_bias = -stepLen*scale*sin(theta)
    // ∂x'/∂scale = stepLen*cos(theta)
    const F = identity(4);
    F[0][2] = - stepLen * scale * Math.sin(theta);
    F[0][3] = stepLen * Math.cos(theta);
    F[1][2] =   stepLen * scale * Math.cos(theta);
    F[1][3] = stepLen * Math.sin(theta);
    // P = F P F^T + Q
    const Ft = transpose(F);
    this.P = matAdd(matMul(matMul(F, this.P), Ft), this.Q);
  }

  // GPS update: lat,lon with accuracy (meters) -> measurement z = [x,y]
  update_gps(lat, lon, accuracy=10) {
    if (!this.origin) return;
    const [mx,my] = this.ll_to_xy(lat, lon);
    // Measurement model: z = H x + noise, H = [[1,0,0,0],[0,1,0,0]]
    const H = [[1,0,0,0],[0,1,0,0]];
    const z = [[mx],[my]];
    const x_vec = [[this.x[0]],[this.x[1]],[this.x[2]],[this.x[3]]];
    // innovation
    const yk = matSub(z, matMul(H, x_vec)); // 2x1
    // S = H P H^T + R
    const Rm = [[accuracy*accuracy,0],[0,accuracy*accuracy]];
    const S = matAdd(matMul(matMul(H, this.P), transpose(H)), Rm);
    // K = P H^T S^-1  (invert 2x2)
    const Sdet = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (Math.abs(Sdet) < 1e-9) return;
    const Sinv = [[ S[1][1]/Sdet, -S[0][1]/Sdet ], [ -S[1][0]/Sdet, S[0][0]/Sdet ]];
    const PHt = matMul(this.P, transpose(H)); // 4x2
    const K = matMul(PHt, Sinv); // 4x2
    // x = x + K * yk
    const Kyk = matMul(K, yk); // 4x1
    for (let i=0;i<4;i++) this.x[i] += Kyk[i][0];
    // P = (I - K H) P
    const KH = matMul(K, H); // 4x4
    const I = identity(4);
    const IKH = matSub(I, KH);
    this.P = matMul(IKH, this.P);
  }

  // convenience getters
  getStateMeters() { return { x: this.x[0], y: this.x[1], theta_bias: this.x[2], scale: this.x[3] }; }
  getLatLon() {
    if (!this.origin) return null;
    return this.xy_to_ll(this.x[0], this.x[1]);
  }
}

window.SimpleEKF = SimpleEKF;
