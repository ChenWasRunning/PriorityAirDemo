const { test } = require('node:test');
const assert = require('node:assert/strict');
const { AirSimulation } = require('./simulation');
test('active-drone means use OD projection and handle an empty airspace', () => {
  const sim = new AirSimulation(seeded());
  assert(Number.isNaN(sim.means().v));
  sim.drones = [
    { origin: { x: 0, y: 0 }, destination: { x: 300, y: 400 }, distance: 500, duration: 25 },
    { origin: { x: 200, y: 0 }, destination: { x: 0, y: 0 }, distance: 200, duration: 10 }
  ];
  assert.deepEqual(sim.means(), { v: 20, u: 20, s: 350 });
  sim.reset();
  assert(Number.isNaN(sim.means().s));
});
function seeded(seed = 42) {
  return () => { seed = (1664525 * seed + 1013904223) >>> 0; return seed / 4294967296; };
}
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
