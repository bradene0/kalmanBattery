import React, { useState, useEffect, useRef } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { Battery, Play, Pause, RotateCcw } from 'lucide-react';

class KalmanFilter {
  constructor() {
    this.x = [100, 10];
    
    this.P = [
      [25, 0],
      [0, 1]
    ];
    
    this.Q = [
      [0.1, 0],
      [0, 0.01]
    ];
    
    this.R = [
      [4, 0],
      [0, 0.25]
    ];
    
    this.dt = 1;
  }
  
  // Matrix operations
  multiply(A, B) {
    const result = Array(A.length).fill().map(() => Array(B[0].length).fill(0));
    for (let i = 0; i < A.length; i++) {
      for (let j = 0; j < B[0].length; j++) {
        for (let k = 0; k < B.length; k++) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }
    return result;
  }
  
  transpose(matrix) {
    return matrix[0].map((_, i) => matrix.map(row => row[i]));
  }
  
  add(A, B) {
    return A.map((row, i) => row.map((val, j) => val + B[i][j]));
  }
  
  subtract(A, B) {
    return A.map((row, i) => row.map((val, j) => val - B[i][j]));
  }
  
  inverse2x2(matrix) {
    const [[a, b], [c, d]] = matrix;
    const det = a * d - b * c;
    return [
      [d / det, -b / det],
      [-c / det, a / det]
    ];
  }
  
  predict(current_draw) {
    const capacity_ah = this.x[1];
    const soc_change = -(current_draw * this.dt / capacity_ah) * 100;
    
    const F = [
      [1, soc_change / capacity_ah],
      [0, 1]
    ];
    
    const x_pred = [
      this.x[0] + soc_change,
      this.x[1]
    ];
    
    const FT = this.transpose(F);
    const P_pred = this.add(this.multiply(this.multiply(F, this.P), FT), this.Q);
    
    this.x = x_pred;
    this.P = P_pred;
  }
  
  update(voltage, current) {
    const soc = this.x[0];
    const capacity = this.x[1];
    
    const expected_voltage = 3.2 + (soc / 100) * 0.9;
    const expected_current = current;
    
    const H = [
      [0.009, 0],
      [0, 0]
    ];
    
    const z = [voltage, current];
    const h = [expected_voltage, expected_current];
    const y = this.subtract([z], [h])[0];
    
    const HT = this.transpose(H);
    const S = this.add(this.multiply(this.multiply(H, this.P), HT), this.R);
    
    const K = this.multiply(this.multiply(this.P, HT), this.inverse2x2(S));
    
    const Ky = this.multiply(K, [[y[0]], [y[1]]]);
    this.x = [
      this.x[0] + Ky[0][0],
      this.x[1] + Ky[1][0]
    ];
    
    const I = [[1, 0], [0, 1]];
    const I_KH = this.subtract(I, this.multiply(K, H));
    this.P = this.multiply(I_KH, this.P);
    
    this.x[0] = Math.max(0, Math.min(100, this.x[0]));
  }
  
  getState() {
    return {
      soc: this.x[0],
      capacity: this.x[1],
      uncertainty: Math.sqrt(this.P[0][0])
    };
  }
}

export default function BatteryKalmanFilter() {
  const [isRunning, setIsRunning] = useState(false);
  const [time, setTime] = useState(0);
  const [data, setData] = useState([]);
  const kalmanRef = useRef(new KalmanFilter());
  const intervalRef = useRef(null);
  
  const simulateReading = (t) => {
    const baseTime = t / 10;
    
    const trueSoc = Math.max(0, 100 - baseTime * 2);
    const trueVoltage = 3.2 + (trueSoc / 100) * 0.9;
    const trueCurrent = 2.0 + Math.sin(baseTime / 10) * 0.5;
    
    const voltageNoise = (Math.random() - 0.5) * 0.4;
    const currentNoise = (Math.random() - 0.5) * 1.0;
    
    return {
      time: t,
      trueVoltage: trueVoltage,
      trueCurrent: trueCurrent,
      trueSoc: trueSoc,
      measuredVoltage: trueVoltage + voltageNoise,
      measuredCurrent: trueCurrent + currentNoise
    };
  };
  
  const step = () => {
    const reading = simulateReading(time);
    const kf = kalmanRef.current;
    
    kf.predict(reading.measuredCurrent);
    kf.update(reading.measuredVoltage, reading.measuredCurrent);
    
    const state = kf.getState();
    
    const newPoint = {
      time: time,
      trueSoc: reading.trueSoc,
      measuredVoltage: reading.measuredVoltage,
      estimatedSoc: state.soc,
      uncertainty: state.uncertainty,
      voltage: reading.trueVoltage,
      current: reading.trueCurrent
    };
    
    setData(prev => [...prev.slice(-49), newPoint]);
    setTime(prev => prev + 1);
  };
  
  useEffect(() => {
    if (isRunning) {
      intervalRef.current = setInterval(step, 200);
    } else {
      clearInterval(intervalRef.current);
    }
    
    return () => clearInterval(intervalRef.current);
  }, [isRunning, time]);
  
  const reset = () => {
    setIsRunning(false);
    setTime(0);
    setData([]);
    kalmanRef.current = new KalmanFilter();
  };
  
  const currentState = kalmanRef.current.getState();
  const latestData = data[data.length - 1];
  
  return (
    <div className="w-full max-w-6xl mx-auto p-6 bg-white">
      <div className="mb-6">
        <h1 className="text-3xl font-bold text-gray-800 mb-2 flex items-center gap-2">
          <Battery className="text-blue-600" />
          Battery Level Kalman Filter
        </h1>
        <p className="text-gray-600">
          Real-time estimation of battery state-of-charge using voltage and current measurements
        </p>
        
        <div className="mt-4 bg-blue-50 p-4 rounded-lg">
          <h3 className="font-medium text-blue-800 mb-2">Consumer Applications & Benefits</h3>
          <div className="text-sm text-blue-700 space-y-2">
            <p><strong>Smartphones & Laptops:</strong> Provides accurate battery percentage even when voltage readings fluctuate due to temperature changes or aging batteries.</p>
            <p><strong>Electric Vehicles:</strong> Gives drivers reliable range estimates by combining multiple sensors and learning battery degradation patterns over time.</p>
            <p><strong>Power Tools & Drones:</strong> Prevents unexpected shutdowns by filtering sensor noise and predicting remaining runtime under varying loads.</p>
            <p><strong>Key Benefits:</strong> More accurate estimates than simple voltage meters, adapts to battery aging, reduces "battery anxiety" by providing reliable predictions, and extends battery life through better management.</p>
          </div>
        </div>
      </div>
      
      {/* Controls */}
      <div className="mb-6 flex gap-4 items-center">
        <button
          onClick={() => setIsRunning(!isRunning)}
          className={`flex items-center gap-2 px-4 py-2 rounded-lg font-medium ${
            isRunning 
              ? 'bg-red-600 hover:bg-red-700 text-white' 
              : 'bg-green-600 hover:bg-green-700 text-white'
          }`}
        >
          {isRunning ? <Pause size={20} /> : <Play size={20} />}
          {isRunning ? 'Pause' : 'Start'}
        </button>
        
        <button
          onClick={reset}
          className="flex items-center gap-2 px-4 py-2 bg-gray-600 hover:bg-gray-700 text-white rounded-lg font-medium"
        >
          <RotateCcw size={20} />
          Reset
        </button>
        
        <div className="text-sm text-gray-600">
          Time: {time}s
        </div>
      </div>
      
      {/* Status Cards */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4 mb-6">
        <div className="bg-blue-50 p-4 rounded-lg">
          <h3 className="font-medium text-blue-800">Estimated SoC</h3>
          <p className="text-2xl font-bold text-blue-600">
            {currentState.soc.toFixed(1)}%
          </p>
          <p className="text-sm text-blue-600">
            ±{currentState.uncertainty.toFixed(1)}%
          </p>
        </div>
        
        <div className="bg-green-50 p-4 rounded-lg">
          <h3 className="font-medium text-green-800">Capacity</h3>
          <p className="text-2xl font-bold text-green-600">
            {currentState.capacity.toFixed(2)} Ah
          </p>
        </div>
        
        <div className="bg-purple-50 p-4 rounded-lg">
          <h3 className="font-medium text-purple-800">Voltage</h3>
          <p className="text-2xl font-bold text-purple-600">
            {latestData ? latestData.measuredVoltage.toFixed(2) : '--'} V
          </p>
        </div>
        
        <div className="bg-orange-50 p-4 rounded-lg">
          <h3 className="font-medium text-orange-800">Current</h3>
          <p className="text-2xl font-bold text-orange-600">
            {latestData ? latestData.current.toFixed(2) : '--'} A
          </p>
        </div>
      </div>
      
      {/* Charts */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* State of Charge Chart */}
        <div className="bg-gray-50 p-4 rounded-lg">
          <h3 className="font-medium text-gray-800 mb-4">State of Charge Estimation</h3>
          <ResponsiveContainer width="100%" height={300}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" />
              <XAxis dataKey="time" />
              <YAxis domain={[0, 100]} />
              <Tooltip />
              <Legend />
              <Line 
                type="monotone" 
                dataKey="trueSoc" 
                stroke="#10b981" 
                strokeWidth={2}
                name="True SoC"
                dot={false}
              />
              <Line 
                type="monotone" 
                dataKey="estimatedSoc" 
                stroke="#3b82f6" 
                strokeWidth={2}
                name="Kalman Estimate"
                dot={false}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
        
        {/* Voltage Chart */}
        <div className="bg-gray-50 p-4 rounded-lg">
          <h3 className="font-medium text-gray-800 mb-4">Voltage Measurements</h3>
          <ResponsiveContainer width="100%" height={300}>
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" />
              <XAxis dataKey="time" />
              <YAxis domain={[3.0, 4.2]} />
              <Tooltip />
              <Legend />
              <Line 
                type="monotone" 
                dataKey="voltage" 
                stroke="#10b981" 
                strokeWidth={2}
                name="True Voltage"
                dot={false}
              />
              <Line 
                type="monotone" 
                dataKey="measuredVoltage" 
                stroke="#f59e0b" 
                strokeWidth={1}
                name="Measured (Noisy)"
                dot={false}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* Algorithm Description */}
      <div className="mt-6 bg-gray-50 p-4 rounded-lg">
        <h3 className="font-medium text-gray-800 mb-2">How It Works</h3>
        <div className="text-sm text-gray-600 space-y-2">
          <p><strong>State Vector:</strong> [State of Charge %, Battery Capacity Ah]</p>
          <p><strong>Measurements:</strong> Noisy voltage and current readings</p>
          <p><strong>Prediction:</strong> SoC decreases based on current draw and capacity</p>
          <p><strong>Update:</strong> Corrects estimate using voltage-SoC relationship</p>
          <p><strong>Benefits:</strong> Smooths noisy measurements, estimates uncertainty, adapts to battery aging</p>
        </div>
      </div>
    </div>
  );
}
