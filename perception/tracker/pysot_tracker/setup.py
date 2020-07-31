from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

ext_modules = [
    Extension(
        name='toolkit.utils.region',
        sources=[
            'src/toolkit/utils/region.pyx',
            'src/toolkit/utils/src/region.c',
        ],
        include_dirs=[
            'src/toolkit/utils/src'
        ]
    )
]

setup(
    name='toolkit',
    packages=['toolkit'],
    ext_modules=cythonize(ext_modules)
)
