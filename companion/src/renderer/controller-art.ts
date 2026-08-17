import dualSenseArtUrl from '../../../assets/controllers/dualsense-edge-front.svg';
import genericPadArtUrl from '../../../assets/controllers/stellaris-cosmic.png';
import type { ControllerDeviceType } from './controller-devices';

// Front-on artwork for the device hero card and the Devices page cards.
//
// Keyed on controllerType, never on persona. Persona is what the bridge
// presents to Windows, and a third-party pad is always presented as an Xbox
// 360 device -- but so is a DualSense whose persona the user switched by hand,
// so persona cannot tell the two apart. controllerType is what bt.cpp
// classified from the DualSense 0x20 vendor probe over Bluetooth, which is the
// only field that says what hardware is actually in the user's hands.
//
// 'unknown' falls through to the DualSense rendering deliberately: it is the
// value carried before a controller has connected, where the hero card is
// showing a placeholder rather than claiming a specific pad.
export function controllerArtUrl(type: ControllerDeviceType | undefined): string {
  return type === 'generic-hid' ? genericPadArtUrl : dualSenseArtUrl;
}
