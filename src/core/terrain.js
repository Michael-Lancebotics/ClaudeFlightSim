// Beach terrain. NED world: x = North (out to sea), y = East, z = Down.
// Sea level is z = 0. The launch pad is at the origin, 40 m from the water.

export const TERRAIN = {
  shoreBase: 40, // m north of the pad
  padHeight: 1.0, // m above mean sea level
};

/** North coordinate of the waterline at a given East coordinate. */
export function shoreNorth(east) {
  return TERRAIN.shoreBase + 8 * Math.sin(east / 120) + 3 * Math.sin(east / 37 + 1.3);
}

/** Terrain height above mean sea level (m). Negative/zero over water. */
export function groundHeight(north, east) {
  const d = shoreNorth(east) - north; // distance inland from the waterline
  if (d <= 0) return Math.max(-0.5 + d * 0.02, -6); // seabed (below the water surface)
  // wet sand -> dry sand rising to the pad at 40 m, then gentle slope and dunes
  let h;
  if (d < 40) h = (TERRAIN.padHeight * d) / 40;
  else if (d < 100) h = TERRAIN.padHeight + 0.025 * (d - 40);
  else {
    const t = d - 100;
    h = 2.5 + 2.0 * (1 - Math.exp(-t / 25)) + 0.8 * Math.sin(east / 23) * Math.sin(t / 11);
  }
  // keep the pad area flat
  const rPad = Math.hypot(north, east);
  if (rPad < 6) {
    const w = Math.min(1, (6 - rPad) / 3);
    h = h * (1 - w) + TERRAIN.padHeight * w;
  }
  return h;
}

/** True if the point is over open water. */
export function isWater(north, east) {
  return north > shoreNorth(east);
}

/** Height of the solid/liquid surface (ground or water) above sea level. */
export function surfaceHeight(north, east) {
  return isWater(north, east) ? 0 : groundHeight(north, east);
}
