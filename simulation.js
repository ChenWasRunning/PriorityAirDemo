class AirSimulation {
  constructor(random = Math.random) {
    this.random = random;
    this.reset();
  }

  exponential() {
    return -Math.log(1 - Math.min(1 - Number.EPSILON, Math.max(Number.EPSILON, this.random())));
  }

  reset() {
    this.time = 0;
    this.drones = [];
    this.completed = 0;
    this.outflowSecond = -1;
    this.outflow = 0;
    this.activeTime = 0;
    this.activeTimeHistory = [{ time: 0, value: 0 }];
    this.meanAccumulation = 0;
    this.flowHistory = [{ time: 0, g: 0, n: 0 }];
    this.flowLabels = { time: 0, g: 0, n: 0 };
    this.completionHistory = [{ time: 0, count: 0 }];
    this.arrivals = { priority: 0, standard: 0 };
    this.remaining = { priority: this.exponential(), standard: this.exponential() };
  }

  createTrip(kind, entryTime) {
    let origin, destination, distance;
    do {
      origin = { x: this.random() * 500, y: this.random() * 500 };
      destination = { x: this.random() * 500, y: this.random() * 500 };
      distance = Math.hypot(destination.x - origin.x, destination.y - origin.y);
    } while (distance < 100);
    return { kind, entryTime, origin, destination, distance, duration: distance / 20 };
  }

  advance(dt, totalRate, alpha) {
    if (!Number.isFinite(dt) || dt < 0 || !Number.isFinite(totalRate) || totalRate < 0 ||
        !Number.isFinite(alpha) || alpha < 0 || alpha > 1) throw new RangeError('Invalid simulation parameters');
    const target = this.time + dt;
    // Resolve every second even when accelerated frames cross several samples.
    while (this.time < target) {
      const end = Math.min(target, Math.floor(this.time) + 1);
      this.advanceSegment(end - this.time, totalRate, alpha);
      if (Number.isInteger(end)) {
        this.activeTimeHistory.push({ time: end, value: this.activeTime });
        while (this.activeTimeHistory.length > 1 && this.activeTimeHistory[1].time <= end - 300) this.activeTimeHistory.shift();
        this.meanAccumulation = Math.max(0, (this.activeTime - this.activeTimeHistory[0].value) / 300);
        this.completionSummary();
        if (end % 10 === 0) this.flowHistory.push({ time: end, g: this.outflow, n: this.meanAccumulation });
        if (end % 120 === 0) this.flowLabels = { time: end, g: this.outflow, n: this.meanAccumulation };
      }
    }
    while (this.flowHistory.length > 1 && this.flowHistory[1].time <= this.time - 600) this.flowHistory.shift();
  }

  advanceSegment(dt, totalRate, alpha) {
    const end = this.time + dt;
    const completions = [];
    let activeTime = this.drones.length * dt;
    const rates = { priority: totalRate * alpha, standard: totalRate * (1 - alpha) };
    for (const kind of ['priority', 'standard']) {
      const rate = rates[kind];
      if (rate === 0) continue;
      let cursor = this.time;
      // Unit-exponential residuals preserve the arrival process when rates change.
      while (this.remaining[kind] <= rate * (end - cursor)) {
        cursor += this.remaining[kind] / rate;
        const trip = this.createTrip(kind, cursor);
        this.arrivals[kind]++;
        activeTime += end - cursor;
        if (cursor + trip.duration > end) this.drones.push(trip);
        else completions.push(cursor + trip.duration);
        this.remaining[kind] = this.exponential();
      }
      this.remaining[kind] -= rate * (end - cursor);
    }
    this.time = end;
    this.drones = this.drones.filter(trip => {
      const exit = trip.entryTime + trip.duration;
      if (exit > end) return true;
      completions.push(exit);
      return false;
    });
    for (const time of completions.sort((a, b) => a - b)) {
      activeTime -= end - time;
      this.completionHistory.push({ time, count: ++this.completed });
    }
    this.activeTime += activeTime;
    // Keep one anchor before the rolling window to preserve its initial count.
    const start = Math.max(0, end - 600);
    let remove = 0;
    while (remove + 1 < this.completionHistory.length && this.completionHistory[remove + 1].time <= start) remove++;
    if (remove) this.completionHistory.splice(0, remove);
  }

  completionSummary() {
    const start = Math.max(0, this.time - 600);
    let firstCount = this.completionHistory[0].count;
    for (const point of this.completionHistory) {
      if (point.time <= start) firstCount = point.count;
    }
    const second = Math.floor(this.time);
    if (second !== this.outflowSecond) {
      const boundary = second;
      let atBoundary = 0, beforeWindow = 0;
      for (const point of this.completionHistory) {
        if (point.time <= boundary) atBoundary = point.count;
        if (point.time <= boundary - 300) beforeWindow = point.count;
      }
      this.outflow = (atBoundary - beforeWindow) / 300;
      this.outflowSecond = second;
    }
    const padding = Math.max(1, Math.ceil((this.completed - firstCount) * 0.05));
    return { start, end: start + 600, firstCount,
      minCount: Math.max(0, firstCount - padding), maxCount: this.completed + padding,
      outflow: this.outflow };
  }

  position(trip) {
    const fraction = Math.min(1, Math.max(0, (this.time - trip.entryTime) / trip.duration));
    return {
      x: trip.origin.x + fraction * (trip.destination.x - trip.origin.x),
      y: trip.origin.y + fraction * (trip.destination.y - trip.origin.y)
    };
  }
}

if (typeof module !== 'undefined') module.exports = { AirSimulation };
