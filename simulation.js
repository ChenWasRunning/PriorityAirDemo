class AirSimulation {
  constructor(random = Math.random) {
    this.random = random;
    this.control = Object.freeze({ detectionRadius: 50, maxSpeed: 15, step: 0.05, repulsion: 100, lateral: 0.6, arrivalRadius: 0.25 });
    this.reset();
  }

  reset() {
    this.time = 0;
    this.nextId = 1;
    this.neighborChecks = 0;
    this.drones = [];
    this.completed = 0;
    this.outflowSecond = -1;
    this.outflow = 0;
    this.completedLengths = [];
    this.departureDistances = [];
    this.kinematicTotals = {
      activeTime: 0, trueDistance: 0, projectedDistance: 0,
      activeTimePriority: 0, trueDistancePriority: 0, projectedDistancePriority: 0,
      activeTimeStandard: 0, trueDistanceStandard: 0, projectedDistanceStandard: 0
    };
    this.kinematicHistory = [{ time: 0, ...this.kinematicTotals }];
    this.estimateHistory = [{ time: 0, U: 0, V: 0, Up: 0, Vp: 0, Us: 0, Vs: 0,
      S: null, L: null, Sp: null, Lp: null, Ss: null, Ls: null,
      g0: 0, gproj: 0, gconv: null }];
    this.estimateLabels = this.estimateHistory[0];
    this.completedByClass = { priority: 0, standard: 0 };
    this.classOutflow = { gp: 0, gs: 0 };
    this.classFlowHistory = [{ time: 0, gp: 0, gs: 0, np: 0, ns: 0 }];
    this.classFlowLabels = { time: 0, gp: 0, gs: 0, np: 0, ns: 0 };
    this.classActiveTime = { priority: 0, standard: 0 };
    this.classAccumulation = { np: 0, ns: 0 };
    this.activeTime = 0;
    this.activeTimeHistory = [{ time: 0, value: 0, priority: 0, standard: 0 }];
    this.meanAccumulation = 0;
    this.flowHistory = [{ time: 0, g: 0, n: 0 }];
    this.flowLabels = { time: 0, g: 0, n: 0 };
    this.completionHistory = [{ time: 0, count: 0 }];
    this.arrivals = { priority: 0, standard: 0 };
    this.entryRates = { priority: 0, standard: 0 };
    this.nextEntry = { priority: Infinity, standard: Infinity };
  }

  createTrip(kind, entryTime) {
    let origin, destination, distance;
    do {
      origin = { x: this.random() * 500, y: this.random() * 500 };
      destination = { x: this.random() * 500, y: this.random() * 500 };
      distance = Math.hypot(destination.x - origin.x, destination.y - origin.y);
    } while (distance < 100);
    return this.makeTrip(kind, entryTime, origin, destination);
  }

  makeTrip(kind, entryTime, origin, destination) {
    return { id: this.nextId++, kind, entryTime, origin, destination,
      distance: Math.hypot(destination.x - origin.x, destination.y - origin.y),
      x: origin.x, y: origin.y, vx: 0, vy: 0, realizedLength: 0,
      trail: [{ x: origin.x, y: origin.y }], trailSpacing: 1 };
  }

  advance(dt, totalRate, alpha) {
    if (!Number.isFinite(dt) || dt < 0 || !Number.isFinite(totalRate) || totalRate < 0 ||
        !Number.isFinite(alpha) || alpha < 0 || alpha > 1) throw new RangeError('Invalid simulation parameters');
    const target = this.time + dt;
    // Resolve every second even when accelerated frames cross several samples.
    while (this.time < target) {
      const end = Math.min(target, (Math.floor(this.time / this.control.step + 1e-7) + 1) * this.control.step, Math.floor(this.time) + 1);
      this.advanceSegment(end - this.time, totalRate, alpha);
      if (Number.isInteger(end)) {
        this.kinematicHistory.push({ time: end, ...this.kinematicTotals });
        while (this.kinematicHistory.length > 1 && this.kinematicHistory[1].time <= end - 300) this.kinematicHistory.shift();
        this.activeTimeHistory.push({ time: end, value: this.activeTime, ...this.classActiveTime });
        while (this.activeTimeHistory.length > 1 && this.activeTimeHistory[1].time <= end - 300) this.activeTimeHistory.shift();
        this.meanAccumulation = Math.max(0, (this.activeTime - this.activeTimeHistory[0].value) / 300);
        this.classAccumulation = {
          np: Math.max(0, (this.classActiveTime.priority - this.activeTimeHistory[0].priority) / 300),
          ns: Math.max(0, (this.classActiveTime.standard - this.activeTimeHistory[0].standard) / 300)
        };
        this.completionSummary();
        const estimates = this.outflowEstimates();
        if (end % 10 === 0) this.estimateHistory.push({ time: end, ...estimates });
        if (end % 120 === 0) this.estimateLabels = { time: end, ...estimates };
        if (end % 10 === 0) this.flowHistory.push({ time: end, g: this.outflow, n: this.meanAccumulation });
        if (end % 120 === 0) this.flowLabels = { time: end, g: this.outflow, n: this.meanAccumulation };
        if (end % 10 === 0) this.classFlowHistory.push({ time: end, ...this.classOutflow, ...this.classAccumulation });
        if (end % 120 === 0) this.classFlowLabels = { time: end, ...this.classOutflow, ...this.classAccumulation };
      }
    }
    while (this.flowHistory.length > 1 && this.flowHistory[1].time <= this.time - 600) this.flowHistory.shift();
    while (this.estimateHistory.length > 1 && this.estimateHistory[1].time <= this.time - 600) this.estimateHistory.shift();
    while (this.classFlowHistory.length > 1 && this.classFlowHistory[1].time <= this.time - 600) this.classFlowHistory.shift();
  }

  advanceSegment(dt, totalRate, alpha) {
    const end = this.time + dt;
    const entries = [];
    const rates = { priority: totalRate * alpha, standard: totalRate * (1 - alpha) };
    for (const kind of ['priority', 'standard']) {
      const rate = rates[kind];
      // A changed rate starts a fresh fixed interval from the change time.
      if (rate !== this.entryRates[kind]) {
        this.entryRates[kind] = rate;
        this.nextEntry[kind] = rate > 0 ? this.time + 1 / rate : Infinity;
      }
      if (rate === 0) continue;
      while (this.nextEntry[kind] <= end + 1e-10) {
        const cursor = Math.max(this.time, Math.min(end, this.nextEntry[kind]));
        const trip = this.createTrip(kind, cursor);
        this.arrivals[kind]++;
        entries.push(trip);
        this.nextEntry[kind] += 1 / rate;
      }
    }
    for (const trip of entries.sort((a, b) => a.entryTime - b.entryTime)) {
      this.moveDrones(trip.entryTime - this.time);
      this.time = trip.entryTime;
      this.drones.push(trip);
      this.departureDistances.push({ time: trip.entryTime, distance: trip.distance, kind: trip.kind });
    }
    this.moveDrones(end - this.time);
    this.time = end;
    // Keep one anchor before the rolling window to preserve its initial count.
    const start = Math.max(0, end - 600);
    let remove = 0;
    while (remove + 1 < this.completionHistory.length && this.completionHistory[remove + 1].time <= start) remove++;
    if (remove) this.completionHistory.splice(0, remove);
  }

  velocities(dt) {
    const { detectionRadius: radius, maxSpeed, repulsion, lateral } = this.control;
    const grid = new Map();
    const key = (x, y) => `${x},${y}`;
    for (const drone of this.drones) {
      const cell = key(Math.floor(drone.x / radius), Math.floor(drone.y / radius));
      if (!grid.has(cell)) grid.set(cell, []);
      grid.get(cell).push(drone);
    }
    this.neighborChecks = 0;
    return this.drones.map(drone => {
      const dx = drone.destination.x - drone.x, dy = drone.destination.y - drone.y;
      const distance = Math.hypot(dx, dy);
      const speed = Math.min(maxSpeed, distance / dt);
      let vx = distance > 0 ? dx / distance * speed : 0;
      let vy = distance > 0 ? dy / distance * speed : 0;
      const cx = Math.floor(drone.x / radius), cy = Math.floor(drone.y / radius);
      for (let ix = cx - 1; ix <= cx + 1; ix++) {
        for (let iy = cy - 1; iy <= cy + 1; iy++) {
          for (const other of grid.get(key(ix, iy)) || []) {
            if (other === drone || (drone.kind === 'priority' && other.kind !== 'priority')) continue;
            this.neighborChecks++;
            let rx = drone.x - other.x, ry = drone.y - other.y;
            const squared = rx * rx + ry * ry;
            if (squared >= radius * radius) continue;
            const separation = Math.sqrt(squared);
            if (separation < 1e-8) {
              // Opposite deterministic directions for coincident positions.
              const angle = Math.min(drone.id, other.id) * 2.399963229728653;
              const sign = drone.id < other.id ? 1 : -1;
              rx = Math.cos(angle) * sign; ry = Math.sin(angle) * sign;
            } else { rx /= separation; ry /= separation; }
            // Smooth finite-range potential gradient, no dense pair matrix.
            // Taper distant repulsion on final approach; retain the close core.
            const approachScale = separation > 5 ? Math.min(1, distance / 3) : 1;
            const force = repulsion * (1 - separation / radius) ** 2 * approachScale;
            const approaching = dx * rx + dy * ry < 0;
            vx += force * (rx - (approaching ? lateral * ry : 0));
            vy += force * (ry + (approaching ? lateral * rx : 0));
          }
        }
      }
      const norm = Math.hypot(vx, vy);
      if (norm > maxSpeed) { vx *= maxSpeed / norm; vy *= maxSpeed / norm; }
      return { vx, vy };
    });
  }

  moveDrones(dt) {
    if (dt <= 0 || this.drones.length === 0) return;
    const velocities = this.velocities(dt);
    const survivors = [], completions = [];
    for (let i = 0; i < this.drones.length; i++) {
      const drone = this.drones[i], velocity = velocities[i];
      const nx = Math.max(0, Math.min(500, drone.x + velocity.vx * dt));
      const ny = Math.max(0, Math.min(500, drone.y + velocity.vy * dt));
      const sx = nx - drone.x, sy = ny - drone.y;
      const dx = drone.destination.x - drone.x, dy = drone.destination.y - drone.y;
      const length2 = sx * sx + sy * sy;
      const fraction = length2 > 0 ? Math.max(0, Math.min(1, (dx * sx + dy * sy) / length2)) : 0;
      if (Math.hypot(dx - fraction * sx, dy - fraction * sy) <= this.control.arrivalRadius) {
        const activeDuration = dt * fraction;
        this.activeTime += activeDuration;
        this.classActiveTime[drone.kind] += activeDuration;
        this.recordKinematics(drone, sx * fraction, sy * fraction, activeDuration);
        completions.push({ time: this.time + dt * fraction, kind: drone.kind,
          length: drone.realizedLength + Math.sqrt(length2) * fraction });
        continue;
      }
      this.activeTime += dt;
      this.classActiveTime[drone.kind] += dt;
      this.recordKinematics(drone, sx, sy, dt);
      drone.vx = sx / dt; drone.vy = sy / dt;
      drone.realizedLength += Math.sqrt(length2);
      drone.x = nx; drone.y = ny;
      const last = drone.trail[drone.trail.length - 1];
      if (Math.hypot(nx - last.x, ny - last.y) >= drone.trailSpacing) {
        drone.trail.push({ x: nx, y: ny });
        if (drone.trail.length > 1024) {
          drone.trail = drone.trail.filter((_, j) => j % 2 === 0);
          drone.trailSpacing *= 2;
        }
      }
      survivors.push(drone);
    }
    this.drones = survivors;
    for (const { time, kind, length } of completions.sort((a, b) => a.time - b.time)) {
      this.completedLengths.push({ time, length, kind });
      this.completedByClass[kind]++;
      this.completionHistory.push({ time, count: ++this.completed, ...this.completedByClass });
    }
  }

  recordKinematics(drone, dx, dy, activeDuration) {
    const trueDistance = Math.hypot(dx, dy);
    const odx = drone.destination.x - drone.origin.x;
    const ody = drone.destination.y - drone.origin.y;
    this.kinematicTotals.activeTime += activeDuration;
    this.kinematicTotals.trueDistance += trueDistance;
    this.kinematicTotals.projectedDistance += (dx * odx + dy * ody) / drone.distance;
    const suffix = drone.kind === 'priority' ? 'Priority' : 'Standard';
    this.kinematicTotals[`activeTime${suffix}`] += activeDuration;
    this.kinematicTotals[`trueDistance${suffix}`] += trueDistance;
    this.kinematicTotals[`projectedDistance${suffix}`] += (dx * odx + dy * ody) / drone.distance;
  }

  outflowEstimates() {
    while (this.completedLengths.length && this.completedLengths[0].time <= this.time - 300) this.completedLengths.shift();
    while (this.departureDistances.length && this.departureDistances[0].time <= this.time - 300) this.departureDistances.shift();
    const windowStart = this.time - 300;
    let baseline = this.kinematicHistory[0];
    for (const point of this.kinematicHistory) {
      if (point.time <= windowStart) baseline = point;
      else break;
    }
    const activeTime = this.kinematicTotals.activeTime - baseline.activeTime;
    const n = this.meanAccumulation;
    const V = activeTime > 0 ? (this.kinematicTotals.trueDistance - baseline.trueDistance) / activeTime : 0;
    const U = activeTime > 0 ? (this.kinematicTotals.projectedDistance - baseline.projectedDistance) / activeTime : 0;
    const classSpeed = suffix => {
      const classActiveTime = (this.kinematicTotals[`activeTime${suffix}`] || 0) - (baseline[`activeTime${suffix}`] || 0);
      return {
        V: classActiveTime > 0 ? ((this.kinematicTotals[`trueDistance${suffix}`] || 0) - (baseline[`trueDistance${suffix}`] || 0)) / classActiveTime : 0,
        U: classActiveTime > 0 ? ((this.kinematicTotals[`projectedDistance${suffix}`] || 0) - (baseline[`projectedDistance${suffix}`] || 0)) / classActiveTime : 0
      };
    };
    const prioritySpeed = classSpeed('Priority');
    const standardSpeed = classSpeed('Standard');
    const meanDistance = (records, kind) => {
      const selected = kind ? records.filter(record => record.kind === kind) : records;
      return selected.length ? selected.reduce((sum, record) => sum + (record.distance ?? record.length), 0) / selected.length : null;
    };
    const S = meanDistance(this.departureDistances);
    const Sp = meanDistance(this.departureDistances, 'priority');
    const Ss = meanDistance(this.departureDistances, 'standard');
    const L = this.time <= 300
      ? S
      : meanDistance(this.completedLengths);
    const Lp = this.time <= 300 ? Sp : meanDistance(this.completedLengths, 'priority');
    const Ls = this.time <= 300 ? Ss : meanDistance(this.completedLengths, 'standard');
    return { n, U, V, Up: prioritySpeed.U, Vp: prioritySpeed.V,
      Us: standardSpeed.U, Vs: standardSpeed.V, S, L, Sp, Lp, Ss, Ls, g0: this.outflow,
      gproj: n && S > 0 ? n * U / S : 0, gconv: n && L > 0 ? n * V / L : (n ? null : 0) };
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
      let priority = 0, standard = 0, priorPriority = 0, priorStandard = 0;
      for (const point of this.completionHistory) {
        if (point.time <= boundary) atBoundary = point.count;
        if (point.time <= boundary - 300) beforeWindow = point.count;
        if (point.time <= boundary) { priority = point.priority || 0; standard = point.standard || 0; }
        if (point.time <= boundary - 300) { priorPriority = point.priority || 0; priorStandard = point.standard || 0; }
      }
      this.outflow = (atBoundary - beforeWindow) / 300;
      this.classOutflow = { gp: (priority - priorPriority) / 300, gs: (standard - priorStandard) / 300 };
      this.outflowSecond = second;
    }
    const padding = Math.max(1, Math.ceil((this.completed - firstCount) * 0.05));
    return { start, end: start + 600, firstCount,
      minCount: Math.max(0, firstCount - padding), maxCount: this.completed + padding,
      outflow: this.outflow };
  }

  position(trip) {
    return { x: trip.x, y: trip.y };
  }
}

if (typeof module !== 'undefined') module.exports = { AirSimulation };
