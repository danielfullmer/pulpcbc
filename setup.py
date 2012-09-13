import shlex
import subprocess

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

proc = subprocess.Popen(['pkg-config', 'cbc', '--cflags'],
                        stdout=subprocess.PIPE)
compile_args, err = proc.communicate()
compile_args = shlex.split(compile_args)

proc = subprocess.Popen(['pkg-config', 'cbc', '--libs'],
                        stdout=subprocess.PIPE)
link_args, err = proc.communicate()
link_args = shlex.split(link_args)

ext_modules = [Extension('pulpcbc',
                         extra_compile_args=['-fpermissive'] + compile_args,
                         extra_link_args=link_args,
                         language="c++",
                        )]

setup(
    name='PuLP-CBC',
    version='0.1',
    cmdclass={'build_ext': build_ext},
    ext_modules=ext_modules,
)
