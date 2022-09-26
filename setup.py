from setuptools import setup 

setup(
    name='meshcat_mujoco',
    version='0.1.0',    
    description='A example Python package',
    url='https://github.com/ialab-yale/meshcat-mujoco',
    author='Ian Abraham',
    author_email='ian.abraham@yale.edu',
    license='BSD 2-clause',
    packages=['meshcat_mujoco'],
    install_requires=[
                        'meshcat',
                        'dm_control',
                        'numpy',                     
                        'mujoco'
                      ],

    classifiers=[
        'Development Status :: 1 - Planning',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',  
        'Operating System :: POSIX :: Linux',        
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ]
)
