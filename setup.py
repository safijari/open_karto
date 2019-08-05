from setuptools import setup
import os
from pybind11_cmake import CMakeBuild, CMakeExtension

__version__ = 'dev'

commit_var = 'COMMIT'
tag_name_var = 'TAG'

if commit_var in os.environ and os.environ[commit_var]:
    __version__ = "0.0.0-" + os.environ[commit_var]

if tag_name_var in os.environ and os.environ[tag_name_var]:
    __version__ = os.environ[tag_name_var]

setup(
    name='open_karto',
    version=__version__,
    author='Jariullah Safi',
    author_email='safijari@isu.edu',
    ext_modules=[CMakeExtension('open_karto')],
    setup_requires=['pybind11-cmake'],
    install_requires=['pybind11-cmake'],
    cmdclass={'build_ext': CMakeBuild},
    zip_safe=False,
)
