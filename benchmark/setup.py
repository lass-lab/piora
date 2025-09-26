
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext
import pybind11

ext_modules = [
    Pybind11Extension(
        "vectorssd",
        [
            "src/python_binding.cpp",
            "src/vectorssd.cc",
        ],
        include_dirs=[
            pybind11.get_include(),
        ],
        language='c++',
        cxx_std=17,
    ),
]

setup(
    name="vectorssd",
    version="0.1.0",
    author="Donghyun Min",
    author_email="tmxosqkdizid@gmail.com",
    description="Piora Python bindings",
    long_description="Python bindings for Piora operations",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.6",
    install_requires=[
        "numpy>=1.15.0",
    ],
)
