// The SITL JSON bridge, exercised by a mock ArduPilot client.
import test from 'node:test';
import assert from 'node:assert/strict';
import dgram from 'node:dgram';
import { createBridge } from '../tools/sitl-bridge.mjs';

function packet(frame, pwm, rate = 400) {
  const b = Buffer.alloc(8 + 32);
  b.writeUInt16LE(18458, 0);
  b.writeUInt16LE(rate, 2);
  b.writeUInt32LE(frame, 4);
  for (let i = 0; i < 16; i++) b.writeUInt16LE(pwm[i] ?? 0, 8 + 2 * i);
  return b;
}

test('bridge answers servo packets with ArduPilot JSON state and flies', async () => {
  const br = await createBridge({ port: 0, log: () => {} });
  const client = dgram.createSocket('udp4');
  await new Promise((r) => client.bind(0, r));
  const replies = [];
  client.on('message', (m) => replies.push(m.toString()));
  const send = (frame, pwm) => new Promise((res) => {
    const n = replies.length;
    client.send(packet(frame, pwm), br.port, '127.0.0.1');
    const t = setInterval(() => { if (replies.length > n) { clearInterval(t); res(replies[replies.length - 1]); } }, 1);
  });
  // 2 s at idle then 4 s with the lift motors high
  let frame = 0, last;
  const pwm = [1500, 1500, 1000, 1500, 1100, 1100, 1100, 1100, 1000];
  for (let i = 0; i < 800; i++) last = await send(++frame, pwm);
  const atRest = JSON.parse(last);
  assert.ok(Math.abs(atRest.imu.accel_body[2] + 9.8) < 0.3, `accel at rest ${atRest.imu.accel_body}`);
  const climb = [1500, 1500, 1000, 1500, 1850, 1850, 1850, 1850, 1000];
  for (let i = 0; i < 1600; i++) last = await send(++frame, climb);
  const s = JSON.parse(last);
  for (const k of ['timestamp', 'imu', 'position', 'quaternion', 'velocity']) assert.ok(k in s, `missing ${k}`);
  assert.ok(last.endsWith('\n'));
  assert.ok(s.position[2] < -2, `climbed: z=${s.position[2]}`);
  assert.ok(Math.abs(s.timestamp - 6) < 0.01, `timestamp ${s.timestamp}`);
  // duplicate frame -> identical reply, no physics step
  const dup = await send(frame, climb);
  assert.equal(dup, last);
  // gripper releases the payload
  await send(++frame, [...climb.slice(0, 8), 1900]);
  assert.equal(br.vehicle.payloadAttached, false);
  client.close();
  br.close();
});
