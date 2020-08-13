#!/usr/bin/env python

import sys
import rospkg
import time
from rqt_gui.main import Main


def add_arguments(parser):
    group = parser.add_argument_group('Options for rqt_compass plugin')
    group.add_argument('topic', nargs='?', help='The topic name to subscribe to')

main = Main()
sys.exit(main.main(
    sys.argv,
    standalone='local_compass/MyPlugin',
    plugin_argument_provider=add_arguments))
