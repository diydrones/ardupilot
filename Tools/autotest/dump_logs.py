#!/usr/bin/env python
# dump flash logs from SITL
# Andrew Tridgell, April 2013

import pexpect, os, sys, shutil, atexit
import optparse, fnmatch, time, glob, traceback, signal

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pysim'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'mavlink', 'pymavlink'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'mavlink', 'pymavlink', 'generator'))
import util

os.environ['PYTHONUNBUFFERED'] = '1'

def dump_logs(atype):
    '''dump DataFlash logs'''
    logfile = '%s.flashlog' % atype
    print("Dumping logs for %s to %s" % (atype, logfile))
    sil = util.start_SIL(atype)
    log = open(logfile, mode='w')
    mavproxy = util.start_MAVProxy_SIL(atype, setup=True, logfile=log)
    mavproxy.send('\n\n\n')
    print("navigating menus")
    mavproxy.expect(']')
    mavproxy.send("logs\n")
    mavproxy.expect("logs enabled:")
    lognums = []
    i = mavproxy.expect(["No logs", "(\d+) logs"])
    if i == 0:
        numlogs = 0
    else:
        numlogs = int(mavproxy.match.group(1))
    for i in range(numlogs):
        mavproxy.expect("Log (\d+),")
        lognums.append(int(mavproxy.match.group(1)))
    mavproxy.expect("Log]")
    for i in range(numlogs):
        print("Dumping log %u (i=%u)" % (lognums[i], i))
        mavproxy.send("dump %u\n" % lognums[i])
        mavproxy.expect("logs enabled:", timeout=120)
        mavproxy.expect("Log]")
    util.pexpect_close(mavproxy)
    util.pexpect_close(sil)
    log.close()
    print("Saved log for %s to %s" % (atype, logfile))
    return True

dump_logs('ArduCopter')

