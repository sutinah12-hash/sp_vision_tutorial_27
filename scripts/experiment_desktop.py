#!/usr/bin/env python3
"""Temporarily arrange a GNOME desktop for non-overlapping native recordings."""
import argparse
import json
from pathlib import Path
from gi.repository import Gio, GLib


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--save', type=Path)
    parser.add_argument('--restore', type=Path)
    args = parser.parse_args()
    if bool(args.save) == bool(args.restore):
        parser.error('Specify exactly one of --save or --restore')
    proxy = Gio.DBusProxy.new_for_bus_sync(
        Gio.BusType.SESSION, Gio.DBusProxyFlags.NONE, None,
        'org.gnome.Mutter.DisplayConfig', '/org/gnome/Mutter/DisplayConfig',
        'org.gnome.Mutter.DisplayConfig', None)
    state = proxy.call_sync('GetCurrentState', None, Gio.DBusCallFlags.NONE, -1, None).unpack()
    serial, monitors, logical, properties = state
    if len(monitors) != 1 or len(logical) != 1:
        raise RuntimeError('This experiment helper supports the single-monitor VM only')
    connector = monitors[0][0][0]
    modes = monitors[0][1]
    current = next(mode[0] for mode in modes if mode[-1].get('is-current'))
    if args.restore:
        saved = json.loads(args.restore.read_text())
        if saved['connector'] != connector:
            raise RuntimeError('Monitor differs from the saved desktop')
        desired, scale = saved['mode'], saved['scale']
    else:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        with args.save.open('x') as stream:
            json.dump({'connector': connector, 'mode': current, 'scale': logical[0][2]}, stream, indent=2)
        desired = next(mode[0] for mode in modes if mode[1:3] == (2560, 1600))
        scale = 1.0
    layout = [(0, 0, scale, 0, True, [(connector, desired, {})])]
    params = GLib.Variant('(uua(iiduba(ssa{sv}))a{sv})',
                          (serial, 1, layout, {}))
    proxy.call_sync('ApplyMonitorsConfig', params, Gio.DBusCallFlags.NONE, -1, None)
    print(f'{current} -> {desired} (temporary desktop layout only)')


if __name__ == '__main__':
    main()
