export class ChartPlotter {
  constructor(canvasId, bufferSize = 120) {
    this.canvas = document.getElementById(canvasId);
    if (!this.canvas) {
      console.warn(`Chart canvas #${canvasId} not found.`);
      return;
    }
    
    this.ctx = this.canvas.getContext('2d');
    this.bufferSize = bufferSize;
    
    // Circular buffers
    this.targetBuffer = [];
    this.actualBuffer = [];
    
    // Initialize buffers with null or zeros
    for (let i = 0; i < this.bufferSize; i++) {
      this.targetBuffer.push(0.0);
      this.actualBuffer.push(0.0);
    }
  }

  push(targetVal, actualVal) {
    if (!this.canvas) return;
    
    // Slide buffers
    this.targetBuffer.shift();
    this.targetBuffer.push(targetVal);
    
    this.actualBuffer.shift();
    this.actualBuffer.push(actualVal);
    
    this.draw();
  }

  draw() {
    if (!this.ctx) return;
    
    const w = this.canvas.width;
    const h = this.canvas.height;
    const ctx = this.ctx;
    
    // Clear canvas
    ctx.clearRect(0, 0, w, h);
    
    // 1. Find dynamic min/max value bounds to scale vertical axis
    let minVal = Math.min(...this.targetBuffer, ...this.actualBuffer);
    let maxVal = Math.max(...this.targetBuffer, ...this.actualBuffer);
    
    // Add small padding to prevent clipping
    const diff = maxVal - minVal;
    if (diff < 1.0) {
      minVal -= 1.0;
      maxVal += 1.0;
    } else {
      minVal -= diff * 0.15;
      maxVal += diff * 0.15;
    }
    
    const range = maxVal - minVal;
    
    // 2. Draw Grid Lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.lineWidth = 1;
    
    // Horizontal grids (4 lines)
    for (let i = 0; i <= 4; i++) {
      const y = (h - 10) * (i / 4) + 5;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
    
    // Vertical grids (6 lines)
    for (let i = 0; i <= 6; i++) {
      const x = w * (i / 6);
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    
    // Helper to map value to Y canvas pixel coordinate
    const mapY = (val) => {
      // 0 is top, h is bottom. We invert Y so higher values are at top.
      const ratio = (val - minVal) / range;
      return h - 5 - ratio * (h - 10);
    };
    
    // 3. Draw Target Line (Cyan)
    ctx.beginPath();
    ctx.strokeStyle = '#00f2fe';
    ctx.lineWidth = 1.5;
    for (let i = 0; i < this.bufferSize; i++) {
      const x = (w / (this.bufferSize - 1)) * i;
      const y = mapY(this.targetBuffer[i]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    
    // 4. Draw Actual Line (Green with subtle glow shadow)
    ctx.beginPath();
    ctx.strokeStyle = '#39ff14';
    ctx.lineWidth = 2.0;
    
    // Add a glowing shadow effect to actual coordinates for cyberpunk look
    ctx.shadowColor = 'rgba(57, 255, 20, 0.35)';
    ctx.shadowBlur = 4;
    
    for (let i = 0; i < this.bufferSize; i++) {
      const x = (w / (this.bufferSize - 1)) * i;
      const y = mapY(this.actualBuffer[i]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    
    // Reset shadow parameters for other elements
    ctx.shadowColor = 'transparent';
    ctx.shadowBlur = 0;
  }
}
