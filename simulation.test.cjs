const { test } = require('node:test');
const assert = require('node:assert/strict');
const { AirSimulation } = require('./simulation');
test('15 m detection and class-specific yielding, with a 20 m/s speed cap', () => {
  const sim = new AirSimulation(seeded());
  const p = sim.makeTrip('priority', 0, { x: 200, y: 250 }, { x: 400, y: 250 });
  const s = sim.makeTrip('standard', 0, { x: 210, y: 250 }, { x: 0, y: 250 });
  sim.drones = [p, s];
  let v = sim.velocities(0.05);
  assert.deepEqual(v[0], { vx: 20, vy: 0 });
  assert(v[1].vy !== 0);
  s.kind = 'priority';
  v = sim.velocities(0.05);
  assert(v[0].vy !== 0 && v[1].vy !== 0);
  for (const velocity of v) assert(Math.hypot(velocity.vx, velocity.vy) <= 20 + 1e-10);
  s.x = 216;
  assert.deepEqual(sim.velocities(0.05), [{ vx: 20, vy: 0 }, { vx: -20, vy: 0 }]);
});
test('head-on encounters detour and complete without symmetric deadlock', () => {
  for (const kinds of [['standard', 'standard'], ['priority', 'priority'], ['priority', 'standard']]) {
    const sim = new AirSimulation(seeded());
    const first = sim.makeTrip(kinds[0], 0, { x: 200, y: 250 }, { x: 300, y: 250 });
    const second = sim.makeTrip(kinds[1], 0, { x: 300, y: 250 }, { x: 200, y: 250 });
    sim.drones = [first, second];
    let minimum = Infinity;
    for (let i = 0; i < 400; i++) {
      sim.advance(0.05, 0, 0);
      if (sim.drones.length === 2) minimum = Math.min(minimum, Math.hypot(first.x - second.x, first.y - second.y));
      if (kinds[0] !== kinds[1]) assert.equal(first.y, 250);
    }
    assert(minimum > 4);
    assert.equal(sim.completed, 2);
    assert(second.trail.some(p => Math.abs(p.y - 250) > 1));
  }
});
test('coincident starts stay finite and local searches skip distant drones', () => {
  const sim = new AirSimulation(seeded());
  sim.drones = [sim.makeTrip('standard', 0, { x: 250, y: 250 }, { x: 500, y: 250 }), sim.makeTrip('priority', 0, { x: 250, y: 250 }, { x: 0, y: 250 })];
  sim.advance(1, 0, 0);
  for (const p of sim.drones) assert(Number.isFinite(p.x + p.y + p.vx + p.vy));
  assert(Math.hypot(sim.drones[0].x - sim.drones[1].x, sim.drones[0].y - sim.drones[1].y) > 0);
  sim.reset();
  for (let x = 30; x < 500; x += 60) for (let y = 30; y < 500; y += 60) sim.drones.push(sim.makeTrip('standard', 0, { x, y }, { x: 500 - x, y: 500 - y }));
  sim.velocities(0.05);
  assert.equal(sim.neighborChecks, 0);
});
test('five-minute accumulation integrates exact fractional active times', () => {
  const sim = new AirSimulation(seeded());
  sim.drones = [sim.makeTrip('standard', 0, { x: 0, y: 50 }, { x: 210, y: 50 }), sim.makeTrip('standard', 0, { x: 0, y: 450 }, { x: 405, y: 450 })];
  sim.advance(30, 0, 0);
  assert(Math.abs(sim.meanAccumulation - 30.75 / 300) < 1e-9);
  sim.advance(280, 0, 0);
  assert(Math.abs(sim.meanAccumulation - 10.75 / 300) < 1e-9);
  sim.advance(20, 0, 0);
  assert.equal(sim.meanAccumulation, 0);
  assert(sim.activeTimeHistory.length <= 301);
  sim.reset();
  assert.equal(sim.activeTime, 0);
  assert.equal(sim.meanAccumulation, 0);
});
test('flow labels capture two-minute boundaries and hold values between updates', () => {
  const sim = new AirSimulation(seeded());
  sim.advance(119.5, 1, 0.5);
  assert.deepEqual(sim.flowLabels, { time: 0, g: 0, n: 0 });
  sim.advance(1, 1, 0.5);
  assert.deepEqual(sim.flowLabels, sim.flowHistory.find(p => p.time === 120));
  const held = { ...sim.flowLabels };
  sim.advance(119, 0, 0);
  assert.deepEqual(sim.flowLabels, held);
  sim.advance(1, 0, 0);
  assert.deepEqual(sim.flowLabels, sim.flowHistory.find(p => p.time === 240));
  sim.reset();
  assert.deepEqual(sim.flowLabels, { time: 0, g: 0, n: 0 });
});
test('completed counts and rolling history remain correct across large time steps', () => {
  const sim = new AirSimulation(seeded());
  sim.advance(1200, 1, 0.5);
  assert.equal(sim.completed + sim.drones.length, sim.arrivals.priority + sim.arrivals.standard);
  assert(sim.completionHistory[0].time <= 600);
  assert(sim.completionHistory[1].time > 600);
  assert.equal(sim.completionHistory.at(-1).count, sim.completed);
  const arrivals = sim.arrivals.priority + sim.arrivals.standard;
  sim.advance(1000, 0, 0);
  assert.equal(sim.completed, arrivals);
  assert.equal(sim.completionHistory.length, 1);
  sim.reset();
  assert.equal(sim.completed, 0);
  assert.deepEqual(sim.completionHistory, [{ time: 0, count: 0 }]);
});
function seeded(seed = 42) {
  return () => { seed = (1664525 * seed + 1013904223) >>> 0; return seed / 4294967296; };
}
test('outflow changes on simulated seconds and excludes later completions', () => {
  const sim = new AirSimulation(seeded());
  sim.completionHistory = [{ time: 0, count: 0 }, { time: 50, count: 3 }, { time: 65, count: 6 }];
  sim.time = 49.9;
  assert.equal(sim.completionSummary().outflow, 0);
  sim.time = 50;
  assert.equal(sim.completionSummary().outflow, 3 / 300);
  sim.time = 64.9;
  assert.equal(sim.completionSummary().outflow, 3 / 300);
  sim.time = 65;
  assert.equal(sim.completionSummary().outflow, 6 / 300);
  sim.reset();
  assert.equal(sim.completionSummary().outflow, 0);
});
test('accelerated advances record exact ten-second outflow and accumulation samples', () => {
  const sim = new AirSimulation(seeded());
  sim.advance(35.7, 1, 0.5);
  assert.deepEqual(sim.flowHistory.map(p => p.time), [0, 10, 20, 30]);
  assert.equal(sim.outflowSecond, 35);
  const reference = new AirSimulation(seeded());
  for (let i = 1; i <= 3; i++) {
    reference.advance(10, 1, 0.5);
    assert(Math.abs(sim.flowHistory[i].n - reference.activeTime / 300) < 1e-9);
    assert.equal(sim.flowHistory[i].g, reference.completed / 300);
  }
  sim.advance(1000, 0, 0);
  assert(sim.flowHistory.length <= 62);
  assert.equal(sim.flowHistory.at(-1).n, 0);
  assert.equal(sim.flowHistory.at(-1).g, 0);
  sim.reset();
  assert.deepEqual(sim.flowHistory, [{ time: 0, g: 0, n: 0 }]);
});
test('chart uses a ten-minute window, local count limits, and five-minute outflow', () => {
  const sim = new AirSimulation(seeded());
  assert.equal(sim.completionSummary().end, 600);
  sim.time = 3960;
  sim.completed = 130;
  sim.completionHistory = [{ time: 3300, count: 100 }, { time: 3600, count: 110 }, { time: 3800, count: 130 }];
  const summary = sim.completionSummary();
  assert.equal(summary.start / 60, 56);
  assert.equal(summary.end / 60, 66);
  assert(summary.minCount > 0 && summary.minCount < 100);
  assert(summary.maxCount > 130);
  assert.equal(summary.outflow, 20 / 300);
  sim.time = 120;
  sim.completed = 10;
  sim.completionHistory = [{ time: 0, count: 0 }, { time: 60, count: 10 }];
  assert.equal(sim.completionSummary().outflow, 10 / 300);
});
test('ten-minute axes slide continuously with fixed two-minute ticks', () => {
  const sim = new AirSimulation(seeded());
  for (const time of [0, 599, 600, 601, 720, 730, 3960, 10000]) {
    sim.time = time;
    const { start, end } = sim.completionSummary();
    const ticks = [];
    for (let t = Math.ceil(start / 120) * 120; t <= end; t += 120) ticks.push(t / 60);
    assert.equal(end - start, 600);
    assert(ticks.length === 5 || ticks.length === 6);
    assert(time >= start && time <= end);
    if (time === 720) assert.deepEqual(ticks, [2, 4, 6, 8, 10, 12]);
  }
  sim.time = 719.99;
  const before = sim.completionSummary();
  sim.time = 720.01;
  const after = sim.completionSummary();
  assert(Math.abs(after.start - before.start - 0.02) < 1e-9);
  assert(Math.abs(after.end - before.end - 0.02) < 1e-9);
});
test('independent exponential streams produce the requested rates', () => {
  const sim = new AirSimulation(seeded());
  sim.advance(10000, 0.8, 0.25);
  assert(Math.abs(sim.arrivals.priority / 10000 - 0.2) < 0.025);
  assert(Math.abs(sim.arrivals.standard / 10000 - 0.6) < 0.03);
  assert(sim.drones.length < 100);
});
test('zero rates, class endpoints, and rate changes', () => {
  const sim = new AirSimulation(seeded());
  sim.advance(100, 0, 0.5);
  assert.deepEqual(sim.arrivals, { priority: 0, standard: 0 });
  sim.advance(100, 1, 0);
  assert.equal(sim.arrivals.priority, 0);
  assert(sim.arrivals.standard > 0);
  const standard = sim.arrivals.standard;
  sim.advance(100, 1, 1);
  assert.equal(sim.arrivals.standard, standard);
  assert(sim.arrivals.priority > 0);
  sim.advance(100, 0, 0);
  assert.equal(sim.drones.length, 0);
});
test('OD geometry, exact speed, landing and reset', () => {
  const sim = new AirSimulation(seeded());
  for (let i = 0; i < 100; i++) {
    const trip = sim.createTrip('standard', 0);
    assert(trip.distance >= 100);
    for (const p of [trip.origin, trip.destination]) assert(p.x >= 0 && p.x <= 500 && p.y >= 0 && p.y <= 500);
    sim.drones = [trip];
    sim.advance(1, 0, 0);
    const p = sim.position(trip);
    assert(Math.abs(Math.hypot(p.x - trip.origin.x, p.y - trip.origin.y) - 20) < 1e-10);
    const previous = sim.completed;
    sim.advance(trip.distance / 20 + 1, 0, 0);
    assert.equal(sim.completed, previous + 1);
  }
  sim.reset();
  assert.equal(sim.time, 0);
  assert.equal(sim.drones.length, 0);
});
