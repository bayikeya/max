// Main app: sensor handling, QR init, step detection, visualization, EKF wiring

// ---- configuration ----
const spots = [
  { name: "13号館前", lat: 35.69302425885399, lon: 140.0508424409975, radius: 8, images:["image0.jpeg","image1.jpeg"], audio:{A:"spot1.ja.mp3",B:"spot1.en.mp3"} },
  { name: "曲がり角右", lat: 35.69283234340133, lon: 140.05091587321536, radius: 8, images:["image3.jpeg"], audio:{A:"spot2.ja.mp3",B:"spot2.en.mp3"} },
  { name: "目的地", lat: 35.693014464267506, lon: 140.0509410980045, radius: 8, images:["image7.jpeg"], audio:{A:"spot3.ja.mp3",B:"spot3.en.mp3"} }
];
// ------------------------

const camera = document.getElementById('camera');
const info = document.getElementById('info');
const image = document.getElementById('image');
const startBtn = document.getElementById('startBtn');
const nextBtn = document.getElementById('nextBtn');
const patternBtn = document.getElementById('patternBtn');
const logDiv = document.getElementById('log');
const audioEl = document.getElementById('audio');

let currentIndex=0, currentSpot=null;
let ekf = new SimpleEKF();
let ahrs = new Madgwick(0.08, 50);
let originSet=false;
let gpsFirst = true;
let qrUsed=false, qrLat=null, qrLon=null;
let headingDeg = null; // from AHRS or deviceorientation
let lastStepTime = 0;
let stepLengthDefault = 0.7; // meters
let stepCount = 0;
let map, imuLayer, gpsLayer;

// setup map
function initMap(){
  map = L.map('map').setView([35.6930,140.0508], 18);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{ maxZoom: 21 }).addTo(map);
  imuLayer = L.polyline([], { color: 'blue' }).addTo(map);
  gpsLayer = L.polyline([], { color: 'red' }).addTo(map);
}

// logging
function log(msg){ console.log(msg); const p=document.createElement('div'); p.textContent = `${new Date().toISOString()} ${msg}`; logDiv.prepend(p); }

// QR param init
function getQRInitialPosition(){
  const params = new URLSearchParams(window.location.search);
  const lat = parseFloat(params.get('lat'));
  const lon = parseFloat(params.get('lon'));
  if (!isNaN(lat) && !isNaN(lon)) {
    qrUsed = true; qrLat = lat; qrLon = lon;
    log(`QR補正を適用: ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
    // set origin and EKF initial state
    if (!originSet) {
      ekf.setOrigin(lat, lon);
      ekf.x[0]=0; ekf.x[1]=0; // authoritatively set local pos = 0,0 at QR
      originSet=true;
    }
    info.innerText = `QR補正済み位置: ${lat.toFixed(6)}, ${lon.toFixed(6)}`;
  }
}

// camera init
navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' }, audio: false })
  .then(s => camera.srcObject = s)
  .catch(e => log('カメラへのアクセスに失敗: '+e.message));

// start button
startBtn.addEventListener('click', () => {
  getQRInitialPosition();
  if (!originSet) {
    info.innerText = "QR無指定。最初のGPSをoriginに使用します。";
  }
  startBtn.style.display='none';
  nextBtn.style.display='inline-block';
  patternBtn.style.display='inline-block';
  audioEl.loop = true;
  if(currentIndex < spots.length) {
    currentSpot = spots[currentIndex];
    image.src = currentSpot.images[0] || '';
    image.style.display='block';
    audioEl.src = currentSpot.audio?.A || '';
    audioEl.play().catch(()=>{/* ignore autoplay issues */});
  }
  // ensure lastUpdateTime for ahrs sampling frequency works
  log('開始しました。センサ待機中...');
});

// DeviceOrientation fallback for heading
if (window.DeviceOrientationEvent) {
  window.addEventListener('deviceorientation', (ev) => {
    if (ev.alpha != null) {
      // alpha might be relative; keep as fallback heading
      headingDeg = (360 - ev.alpha + 360) % 360;
    }
  });
}

// DeviceMotion: use for step detection (acc) and gyro for AHRS
let accBuffer = []; const ACC_BUF_LEN = 25;
if (window.DeviceMotionEvent) {
  window.addEventListener('devicemotion', (ev) => {
    const acc = ev.accelerationIncludingGravity;
    const gyro = ev.rotationRate || {alpha:0,beta:0,gamma:0};
    if (!acc) return;
    // update AHRS (gyro in deg/s, acc in m/s^2)
    ahrs.update(gyro.alpha || 0, gyro.beta || 0, gyro.gamma || 0, acc.x||0, acc.y||0, acc.z||0);
    const euler = ahrs.getEuler();
    headingDeg = euler.yaw; // use AHRS yaw
    // step detection via magnitude peak
    const mag = Math.sqrt((acc.x||0)**2 + (acc.y||0)**2 + (acc.z||0)**2);
    accBuffer.push(mag);
    if (accBuffer.length > ACC_BUF_LEN) accBuffer.shift();
    const mean = accBuffer.reduce((a,b)=>a+b,0)/accBuffer.length;
    const std = Math.sqrt(accBuffer.reduce((a,b)=>a+(b-mean)**2,0)/accBuffer.length);
    // adaptive threshold
    const threshold = mean + Math.max(0.8, std*0.9);
    const now = Date.now();
    if (mag > threshold && now - lastStepTime > 300) {
      lastStepTime = now;
      stepDetected();
    }
  });
}

// Step handling
function stepDetected(){
  stepCount++;
  const theta_rad = (headingDeg||0) * Math.PI/180;
  const stepLen = stepLengthDefault;
  ekf.predict_step(stepLen, theta_rad);
  const pos = ekf.getLatLon ? ekf.getLatLon() : null;
  const state = ekf.getStateMeters();
  // update map polyline
  imuLayer.addLatLng(pos);
  map.panTo(pos);
  log(`step#${stepCount} heading=${(headingDeg||0).toFixed(1)}deg pos=${pos[0].toFixed(6)},${pos[1].toFixed(6)}`);
  checkProximity(state.x, state.y);
}

// GPS watch
if ("geolocation" in navigator) {
  navigator.geolocation.watchPosition((pos) => {
    const glat = pos.coords.latitude, glon = pos.coords.longitude, acc = pos.coords.accuracy || 10;
    // set origin if none
    if (!originSet) {
      if (qrUsed) {
        // compute bias: we prefer QR anchoring; set origin to QR and compute nothing
        ekf.setOrigin(qrLat, qrLon);
        originSet = true;
        log('origin: QR基準を使用');
      } else {
        ekf.setOrigin(glat, glon);
        originSet = true;
        log('origin: GPS最初のfixを使用');
      }
    }
    // If QR used, treat GPS as observation; ekf expects lat/lon obs, will convert
    ekf.update_gps(glat, glon, acc);
    const latlon = ekf.getLatLon();
    // add to gps polyline
    gpsLayer.addLatLng(latlon);
    log(`GPS obs: ${glat.toFixed(6)},${glon.toFixed(6)} acc=${acc} -> fused ${latlon[0].toFixed(6)},${latlon[1].toFixed(6)}`);
    // map center on first fix
    if (gpsLayer.getLatLngs().length === 1) map.setView(latlon, 19);
  }, (err)=> {
    log('GPS error: '+err.message);
  }, { enableHighAccuracy: true, maximumAge: 0, timeout: 5000 });
}

// proximity: check using EKF state (meters) vs spot in meters from origin
function checkProximity(x_m, y_m) {
  // compute each spot's local xy relative to origin
  if (!ekf.origin) return;
  const R = 6378137;
  const spot = spots[currentIndex];
  if (!spot) return;
  // convert spot latlon to xy
  const dLat = (spot.lat - ekf.origin.lat) * Math.PI/180;
  const dLon = (spot.lon - ekf.origin.lon) * Math.PI/180;
  const sx = dLon * R * Math.cos(ekf.origin.lat * Math.PI/180);
  const sy = dLat * R;
  const dx = sx - x_m, dy = sy - y_m;
  const dist = Math.sqrt(dx*dx + dy*dy);
  const range = spot.radius || 8;
  info.innerText = `次のスポット: ${spot.name}\n残距離: ${dist.toFixed(1)} m\n推定位置: ${ (ekf.getLatLon()? ekf.getLatLon()[0].toFixed(6):'') }, ${ (ekf.getLatLon()? ekf.getLatLon()[1].toFixed(6):'') }`;
  if (!currentSpot && dist < range) {
    // arrived
    currentSpot = spot;
    image.src = spot.images[0] || '';
    audioEl.src = spot.audio?.A || '';
    audioEl.play().catch(()=>{});
    log('スポット到達: '+spot.name);
    // show UI to mark complete
    nextBtn.style.display='inline-block';
  }
}

// Next spot button
nextBtn.addEventListener('click', () => {
  if (!currentSpot) return;
  log(`スポット完了: ${currentSpot.name}`);
  // advance
  currentIndex++;
  currentSpot = null;
  nextBtn.style.display='none';
  if (currentIndex >= spots.length) {
    info.innerText = 'すべてのスポットを通過しました。';
    audioEl.pause();
  } else {
    // optionally set EKF state origin to current EKF pos to reduce drift accumulation
    log('次のスポットへ。EKF内部状態維持。');
  }
});

// init on load
window.addEventListener('load', () => {
  initMap();
  getQRInitialPosition();
  log('アプリロード完了。QRパラメータ確認済み。');
});
