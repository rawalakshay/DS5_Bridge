import { describe, expect, it } from 'vitest';
import { controllerArtUrl } from './controller-art';

describe('controllerArtUrl', () => {
  it('gives a generic pad its own artwork instead of the DualSense rendering', () => {
    // The whole point of the lookup: a third-party pad reaches Windows as an
    // Xbox 360 device, so showing a DualSense next to its name is wrong.
    expect(controllerArtUrl('generic-hid')).not.toBe(controllerArtUrl('dualsense'));
  });

  it('keeps both DualSense variants on the DualSense rendering', () => {
    // Edge and base share one piece of art today; asserting they match keeps a
    // future Edge-specific asset from silently changing only one of them.
    expect(controllerArtUrl('dualsense-edge')).toBe(controllerArtUrl('dualsense'));
  });

  it('falls back to the DualSense rendering before a controller is classified', () => {
    // 'unknown' and undefined are the pre-connection states, where the hero
    // card is a placeholder rather than a claim about specific hardware.
    expect(controllerArtUrl('unknown')).toBe(controllerArtUrl('dualsense'));
    expect(controllerArtUrl(undefined)).toBe(controllerArtUrl('dualsense'));
  });

  it('resolves every controller type to a usable asset URL', () => {
    for (const type of ['unknown', 'dualsense', 'dualsense-edge', 'generic-hid'] as const) {
      expect(controllerArtUrl(type)).toMatch(/\.(svg|png)$/);
    }
  });
});
