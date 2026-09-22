const { test } = require('node:test');
const assert = require('node:assert/strict');
const { AirSimulation } = require('./simulation');
test('completed counts and rolling history remain correct across large time steps', () => {
  const sim = new AirSimulation(seeded());
  sim.advance(1200, 1, 0.5);
  assert.equal(sim.completed + sim.drones.length, sim.arrivals.priority + sim.arrivals.standard);
  assert(sim.completionHistory[0].time <= 540);
  assert(sim.completionHistory[1].time > 540);
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
test('outflow changes only on simulated minutes and excludes later completions', () => {
  const sim = new AirSimulation(seeded());
  sim.completionHistory = [{ time: 0, count: 0 }, { time: 50, count: 3 }, { time: 65, count: 6 }];
  sim.time = 59;
  assert.equal(sim.completionSummary().outflow, 0);
  sim.time = 60;
  assert.equal(sim.completionSummary().outflow, 3 / 300);
  sim.time = 119;
  assert.equal(sim.completionSummary().outflow, 3 / 300);
  sim.time = 121;
  assert.equal(sim.completionSummary().outflow, 6 / 300);
  sim.reset();
  assert.equal(sim.completionSummary().outflow, 0);
});
test('chart uses an eleven-minute window, local count limits, and five-minute outflow', () => {
  const sim = new AirSimulation(seeded());
  assert.equal(sim.completionSummary().end, 660);
  sim.time = 3960;
  sim.completed = 130;
  sim.completionHistory = [{ time: 3300, count: 100 }, { time: 3600, count: 110 }, { time: 3800, count: 130 }];
  const summary = sim.completionSummary();
  assert.equal(summary.start / 60, 55);
  assert.equal(summary.end / 60, 66);
  assert(summary.minCount > 0 && summary.minCount < 100);
  assert(summary.maxCount > 130);
  assert.equal(summary.outflow, 20 / 300);
  sim.time = 120;
  sim.completed = 10;
  sim.completionHistory = [{ time: 0, count: 0 }, { time: 60, count: 10 }];
  assert.equal(sim.completionSummary().outflow, 10 / 300);
});
test('eleven-minute axes always contain six two-minute ticks', () => {
  const sim = new AirSimulation(seeded());
  for (const time of [0, 659, 660, 661, 720, 730, 3960, 10000]) {
    sim.time = time;
    const { start, end } = sim.completionSummary();
    const ticks = [];
    for (let t = Math.ceil(start / 120) * 120; t <= end; t += 120) ticks.push(t / 60);
    assert.equal(end - start, 660);
    assert.equal(ticks.length, 6);
    assert(time >= start && time <= end);
    if (time === 720) assert.deepEqual(ticks, [2, 4, 6, 8, 10, 12]);
  }
});
test('independent exponential streams produce the requested rates', () => {
  const sim = new AirSimulation(seeded());
  sim.advance(100000, 0.8, 0.25);
  assert(Math.abs(sim.arrivals.priority / 100000 - 0.2) < 0.006);
  assert(Math.abs(sim.arrivals.standard / 100000 - 0.6) < 0.01);
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
    sim.time = 1;
    const p = sim.position(trip);
    assert(Math.abs(Math.hypot(p.x - trip.origin.x, p.y - trip.origin.y) - 20) < 1e-10);
    sim.time = trip.duration;
    const end = sim.position(trip);
    assert(Math.hypot(end.x - trip.destination.x, end.y - trip.destination.y) < 1e-10);
  }
  sim.reset();
  assert.equal(sim.time, 0);
  assert.equal(sim.drones.length, 0);
});
