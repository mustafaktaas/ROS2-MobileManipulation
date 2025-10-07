import React, { useEffect, useState, useRef } from 'react';
import io from 'socket.io-client';
import { Joystick } from 'react-joystick-component';
import './App.css';

const socket = io('http://localhost:5000', {
  transports: ['websocket'],
  reconnection: true,
  reconnectionAttempts: 10,
  reconnectionDelay: 1000,
  reconnectionDelayMax: 5000,
  timeout: 20000, // Bağlantı zaman aşımı artırıldı
});

function App() {
  const [mapData, setMapData] = useState(null);
  const [odomData, setOdomData] = useState({ x: 0, y: 0 });
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const canvasRef = useRef(null);
  const offScreenCanvasRef = useRef(null);

  useEffect(() => {
    socket.on('connect', () => {
      console.log('Connected to Flask WebSocket');
      setConnectionStatus('Connected');
    });
    socket.on('disconnect', () => {
      console.log('Disconnected from Flask WebSocket');
      setConnectionStatus('Disconnected');
    });
    socket.on('connect_error', (error) => {
      console.error('WebSocket connection error:', error.message, error);
    });
    socket.on('mqtt_data', (data) => {
      console.log('Received MQTT data:', data);
      if (data.topic === 'factory/robot/map') {
        if (data.payload && data.payload.info && Array.isArray(data.payload.data)) {
          console.log('Valid map data received:', data.payload);
          setMapData(data.payload);
        } else {
          console.error('Invalid map data format:', data.payload);
        }
      } else if (data.topic === 'factory/robot/odom') {
        if (data.payload.position && typeof data.payload.position.x === 'number' && typeof data.payload.position.y === 'number') {
          setOdomData({
            x: data.payload.position.x,
            y: data.payload.position.y,
          });
        } else {
          console.error('Invalid odom data format:', data.payload);
        }
      }
    });

    return () => {
      socket.off('connect');
      socket.off('disconnect');
      socket.off('connect_error');
      socket.off('mqtt_data');
    };
  }, []);

  useEffect(() => {
    if (!mapData) {
      console.log('mapData is null, waiting for valid data');
      return;
    }

    const canvas = canvasRef.current;
    const offScreenCanvas = offScreenCanvasRef.current;
    if (!canvas || !offScreenCanvas) return;

    const ctx = offScreenCanvas.getContext('2d');
    const width = mapData.info.width;
    const height = mapData.info.height;
    const scale = Math.min(600 / width, 600 / height); // Maksimum 600px

    offScreenCanvas.width = width * scale;
    offScreenCanvas.height = height * scale;

    console.log('Drawing map with scaled width:', offScreenCanvas.width, 'height:', offScreenCanvas.height);
    ctx.clearRect(0, 0, offScreenCanvas.width, offScreenCanvas.height);
    ctx.scale(scale, scale);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const index = y * width + x;
        const value = mapData.data[index];
        ctx.fillStyle = value === 0 ? 'white' : value === 100 ? 'black' : 'gray';
        ctx.fillRect(x, height - y - 1, 1, 1);
      }
    }
    ctx.scale(1 / scale, 1 / scale); // Ölçeği sıfırla
  }, [mapData]);

  useEffect(() => {
    const canvas = canvasRef.current;
    const offScreenCanvas = offScreenCanvasRef.current;
    if (!canvas || !offScreenCanvas || !mapData) return;

    const ctx = canvas.getContext('2d');
    const width = mapData.info.width;
    const height = mapData.info.height;
    const resolution = mapData.info.resolution;
    const origin = mapData.info.origin.position;
    const scale = Math.min(600 / width, 600 / height);

    canvas.width = width * scale;
    canvas.height = height * scale;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(offScreenCanvas, 0, 0);

    ctx.scale(scale, scale);
    const robotX = (odomData.x - origin.x) / resolution;
    const robotY = (odomData.y - origin.y) / resolution;
    ctx.fillStyle = 'red';
    ctx.beginPath();
    ctx.arc(robotX, height - robotY, 5 / scale, 0, 2 * Math.PI);
    ctx.fill();
    ctx.scale(1 / scale, 1 / scale);
  }, [odomData, mapData]);

  const handleJoystickMove = (event) => {
    const cmd = { linear_x: event.y * 0.5, angular_z: -event.x * 1.0 };
    socket.emit('publish_mqtt', {
      topic: 'factory/robot/cmd_vel',
      message: JSON.stringify(cmd),
    });
  };

  const handleJoystickStop = () => {
    const cmd = { linear_x: 0.0, angular_z: 0.0 };
    socket.emit('publish_mqtt', {
      topic: 'factory/robot/cmd_vel',
      message: JSON.stringify(cmd),
    });
  };

  return (
    <div className="App">
      <h1>Robot MQTT Dashboard</h1>
      <p>WebSocket Status: {connectionStatus}</p>
      <div className="content-container">
        <div className="map-container">
          <h2>Robot Map</h2>
          <div className="canvas-wrapper">
            <canvas ref={canvasRef} className="map-canvas" />
            <canvas ref={offScreenCanvasRef} style={{ display: 'none' }} />
          </div>
        </div>
        <div className="camera-container">
          <h2>Camera Feed</h2>
          <img src="http://localhost:8080/stream?topic=/camera/color/image_raw" alt="Camera Feed" />
          <div className="joystick-container">
            <h2>Robot Control</h2>
            <Joystick
              size={100}
              baseColor="#ccc"
              stickColor="#333"
              move={handleJoystickMove}
              stop={handleJoystickStop}
            />
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;