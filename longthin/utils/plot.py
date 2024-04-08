from ..ltpacket import *
import time
import argparse
from dataclasses import asdict
import pyqtgraph as pg


def main():
    parser = argparse.ArgumentParser(description='A packet plotter')
    parser.add_argument('packet', help='Packet and property to plot', nargs='+')
    parser.add_argument('--duration', '-d', default=None, help='Max duration to plot')
    args = parser.parse_args()
    
    conn = LTZmq()
    start_time = time.time()

    value_dict = {}
    packets = []
    for i, ps in enumerate(args.packet):
        for packet in ps.split(','):
            typ, prop = packet.split('.')
            typ = LTPacketType[typ.upper()]
            prop = prop.lower()
            value_dict[(typ, prop)] = [[], [], None, i]
            packets.append(typ)
    packets = set(packets)

    app = pg.mkQApp()
    w = pg.GraphicsLayoutWidget()
    w.show()
    w.setWindowTitle('LTPlot')
    w.resize(800, 600)
    w.setWindowTitle('LTPlot')
    w.addLabel('LTPlot')
    w.nextRow()
    plots = []

    for _ in range(len(args.packet)):
        plot = w.addPlot()
        plots.append(plot)
        plot.showGrid(x=True, y=True)
        plot.setLabel('left', 'Value')
        plot.setLabel('bottom', 'Time (s)')
        plot.addLegend()
        w.nextRow()

    for i, (k, v) in enumerate(value_dict.items()):
        name = LTPacketType(k[0]).name + '.' + k[1]
        v[2] = plots[v[3]].plot(pen=(i, len(value_dict)), name=name)

    while True:
        while True:
            packet = conn.read()
            if packet is None:
                break
            t = time.time() - start_time

            if packet.type in packets:
                for (typ, prop), (ts, xs, curve, _) in value_dict.items():
                    if packet.type != typ:
                        continue
                    ts.append(t)
                    xs.append(getattr(packet, prop))

            if args.duration is not None:
                for (typ, prop), (ts, xs, curve, _) in value_dict.items():
                    while ts and ts[0] < t - float(args.duration):
                        ts.pop(0)
                        xs.pop(0)

        for (typ, prop), (ts, xs, curve, _) in value_dict.items():
            curve.setData(ts, xs)
        app.processEvents()


if __name__ == "__main__":
    main()
