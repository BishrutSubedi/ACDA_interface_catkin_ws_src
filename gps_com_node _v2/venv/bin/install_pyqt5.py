#!/home/dnc3/catkin_ws/src/osm_maps_node/venv/bin/python
"""
This program installs PyQt5 from source. The steps are:

    1. Download and install SIP
    2. Download and install PyQt5

Remember that Qt5 has to be installed already in your system, and qmake has to
be in the path in order to be invoked from the script. If qmake is not in your
path, you should use the --qmake argument to explicit where it is.

If you are on MAC OS, you can install Qt5 using brew:

    brew install qt5

If you are on LINUX, you should be able to install Qt5 from your distribution
repositories.

WINDOWS is not supported by this script, sorry.

Usage:
    install_pyqt5.py [--qmake QMAKE_PATH]

Options:
    -h --help           Shows this screen
    --qmake QMAKE_PATH  Path to qmake executable

"""
import platform
import wget
import tarfile
import subprocess
import blessings
from docopt import docopt

SIP_VERSION = '4.18.1'
PYQT_VERSION = '5.7'
SIP_NIX_URL = 'http://sourceforge.net/projects/pyqt/files/sip/' + \
    'sip-{0}/sip-{0}.tar.gz'.format(SIP_VERSION)
PYQT_NIX_URL = 'https://sourceforge.net/projects/pyqt/files/PyQt5/' + \
    'PyQt-{0}/PyQt5_gpl-{0}.tar.gz'.format(PYQT_VERSION)


class PlatformNotSupportedException(Exception):

    def __init__(self, plat):
        self._platform = plat

    def __str__(self):
        return "Sorry, your platform '{}' is not supported".format(
            self._platform)


class Installer(object):

    @property
    def name(self):
        return self._name

    @property
    def version(self):
        return self._version

    @property
    def url(self):
        return self._url

    def __init__(self, name, version, url):
        self._name = name
        self._version = version
        self._url = url
        self._filename = ''
        self._folder = ''

    def download(self):
        self._filename = wget.download(self.url)

    def extract(self):
        if self._filename.endswith(".gz"):
            tar = tarfile.open(self._filename, 'r:gz')
            tar.extractall('.')
            self._folder = tar.getnames()[0]

    def configure(self, params=''):
        platform.os.chdir(self._folder)
        subprocess.check_call(['python', 'configure.py'] + params.split())

    def install(self):
        subprocess.check_call(['make'])
        subprocess.check_call(['make', 'install'])

    def cleanup(self):
        platform.os.chdir('../')
        if self._folder or self._filename:
            subprocess.check_call(['rm', '-r', self._folder, self._filename])


def install_helper(installer, config_params='', mac_patch=False):
    t = blessings.Terminal()
    name = '{0.name} {0.version}'.format(installer)
    try:
        print(t.bold("\nDownloading {} from {}...".format(
            name, installer.url)))
        installer.download()
        print(t.bold("\n\nExtracting {}...".format(name)))
        installer.extract()
        print(t.bold('\nConfiguring {}...'.format(name)))
        installer.configure(config_params)
        print(t.bold('\nInstalling {}...'.format(name)))
        installer.install()
    except subprocess.CalledProcessError:
        exit()
    else:
        print(t.bold_green('\n{} installed succesfully...'.format(name)))
    finally:
        print(t.bold('Cleaning up...\n'))
        installer.cleanup()


def check_platform():
    try:
        plat = platform.system()
        if plat == "Darwin" or plat == "Linux":
            return SIP_NIX_URL, PYQT_NIX_URL
        else:
            raise PlatformNotSupportedException(plat)
    except PlatformNotSupportedException as e:
        print(blessings.Terminal().bold_red(str(e)))
        exit()


if __name__ == '__main__':
    arguments = docopt(__doc__)
    sip_url, pyqt_url = check_platform()

    # SIP
    install_helper(Installer('SIP', SIP_VERSION, sip_url))

    # PyQt5
    config_params = ''
    if arguments.get('--qmake'):
        config_params = '--qmake ' + arguments['--qmake'] + ' --verbose'

    install_helper(
        Installer('PyQt5', PYQT_VERSION, pyqt_url),
        config_params
    )
