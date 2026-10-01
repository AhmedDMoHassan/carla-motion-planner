"""
CARLA 0.8.4 (Coursera) setup checker - Course 4 Final Project
Run with Python 3.6 from the Course4FinalProject folder:

    cd C:\\Coursera\\CarlaSimulator\\PythonClient\\Course4FinalProject
    py -3.6 check_carla_setup.py            # offline checks only
    py -3.6 check_carla_setup.py --connect  # also talk to a running CARLA server

For --connect, start the server first in another terminal:
    CarlaUE4.exe /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30
"""
from __future__ import print_function
import os
import sys
import argparse

OK, BAD, WARN = "[ OK ]", "[FAIL]", "[WARN]"
fails = []


def report(tag, msg, fix=None):
    print("%s %s" % (tag, msg))
    if tag == BAD:
        fails.append(msg)
        if fix:
            print("       fix: %s" % fix)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", action="store_true", help="test connection to a running CARLA server")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    pyclient = os.path.abspath(os.path.join(here, ".."))
    root = os.path.abspath(os.path.join(pyclient, ".."))

    print("\n== 1. Python ==")
    v = sys.version_info
    print("     executable: %s" % sys.executable)
    if v[:2] in ((3, 5), (3, 6)):
        report(OK, "Python %d.%d.%d" % v[:3])
    else:
        report(BAD, "Python %d.%d is not supported by the CARLA 0.8.4 client (needs 3.5 or 3.6)" % v[:2],
               "run this script with  py -3.6 check_carla_setup.py")
    if sys.maxsize <= 2 ** 32:
        report(BAD, "32-bit Python detected", "install the 64-bit (x86-64) Python 3.6.8 installer")
    else:
        report(OK, "64-bit Python")

    print("\n== 2. Folder layout ==")
    checks = [
        (os.path.join(root, "CarlaUE4.exe"), "CarlaUE4.exe in the CarlaSimulator root"),
        (os.path.join(root, "requirements.txt"), "requirements.txt in the CarlaSimulator root"),
        (os.path.join(pyclient, "carla"), "PythonClient\\carla package (the 0.8.4 client)"),
        (os.path.join(pyclient, "live_plotter.py"), "PythonClient\\live_plotter.py"),
        (os.path.join(pyclient, "manual_control.py"), "PythonClient\\manual_control.py"),
    ]
    for path, label in checks:
        if os.path.exists(path):
            report(OK, label)
        else:
            report(BAD, "missing: %s  (%s)" % (label, path),
                   "Course4FinalProject must sit directly inside CarlaSimulator\\PythonClient")
    missing = False
    for f in ["module_7.py", "behavioural_planner.py", "collision_checker.py", "local_planner.py",
              "path_optimizer.py", "velocity_planner.py", "controller2d.py", "cutils.py",
              "options.cfg", "course4_waypoints.txt", "stop_sign_params.txt", "parked_vehicle_params.txt"]:
        if not os.path.exists(os.path.join(here, f)):
            report(BAD, "missing project file: %s" % f, "re-extract Course4FinalProject.zip")
            missing = True
    if not missing:
        report(OK, "all Course 4 project files present")

    print("\n== 3. Python packages ==")
    pkgs = [("numpy", "numpy"), ("scipy", "scipy"), ("matplotlib", "matplotlib"),
            ("pygame", "pygame"), ("PIL", "Pillow"), ("google.protobuf", "protobuf"), ("future", "future")]
    for mod, pipname in pkgs:
        try:
            m = __import__(mod, fromlist=["__version__"])
            report(OK, "%-12s %s" % (pipname, getattr(m, "__version__", "")))
        except Exception as e:
            report(BAD, "%s not importable (%s)" % (pipname, e.__class__.__name__),
                   "py -3.6 -m pip install --user %s" % pipname)
    try:
        import matplotlib
        backend = matplotlib.get_backend()
        report(OK if "tk" in backend.lower() or "qt" in backend.lower() else WARN,
               "matplotlib backend: %s (live plotting needs a GUI backend such as TkAgg)" % backend)
    except Exception:
        pass

    print("\n== 4. CARLA client imports (as module_7.py does) ==")
    sys.path.insert(0, pyclient)
    sys.path.insert(0, here)
    for stmt in ["import live_plotter",
                 "from carla import sensor",
                 "from carla.client import make_carla_client, VehicleControl",
                 "from carla.settings import CarlaSettings",
                 "from carla.tcp import TCPConnectionError",
                 "from carla.controller import utils",
                 "import controller2d",
                 "import scipy.optimize, scipy.integrate, scipy.spatial"]:
        try:
            exec(stmt, {})
            report(OK, stmt)
        except Exception as e:
            report(BAD, "%s  ->  %s: %s" % (stmt, e.__class__.__name__, e))

    print("\n== 5. Options ==")
    try:
        import configparser
        cp = configparser.ConfigParser()
        cp.read(os.path.join(here, "options.cfg"))
        d = cp["Demo Parameters"]
        report(OK, "live_plotting=%s  live_plotting_period=%s" % (d.get("live_plotting"), d.get("live_plotting_period")))
    except Exception as e:
        report(BAD, "options.cfg unreadable: %s" % e)

    if args.connect:
        print("\n== 6. Server connection (%s:%d) ==" % (args.host, args.port))
        try:
            from carla.client import make_carla_client
            from carla.settings import CarlaSettings
            with make_carla_client(args.host, args.port, timeout=15) as client:
                s = CarlaSettings()
                s.set(SynchronousMode=True, SendNonPlayerAgentsInfo=True,
                      NumberOfVehicles=0, NumberOfPedestrians=0, WeatherId=1)
                scene = client.load_settings(s)
                n = len(scene.player_start_spots)
                client.start_episode(0)
                meas, _ = client.read_data()
                p = meas.player_measurements.transform.location
                client.send_control(steer=0.0, throttle=0.0, brake=1.0, hand_brake=False, reverse=False)
                report(OK, "connected; map has %d start spots; ego at (%.1f, %.1f, %.1f)" % (n, p.x, p.y, p.z))
                if n < 2:
                    report(WARN, "few start spots - is the server on /Game/Maps/Course4 ?")
        except Exception as e:
            report(BAD, "could not connect/run an episode: %s: %s" % (e.__class__.__name__, e),
                   "start CarlaUE4.exe with -carla-server first, allow it through the firewall, same --port")

    print("\n" + "=" * 60)
    if fails:
        print("%d problem(s) found - fix the [FAIL] lines above, then run again." % len(fails))
        sys.exit(1)
    print("All checks passed." + ("" if args.connect else "  Next: run again with --connect while CARLA is running."))


if __name__ == "__main__":
    main()
