from conan import ConanFile

class InteractiveRateTendons(ConanFile):
    name = 'interactive_rate_tendons'
    version = '1.0.0'
    user = 'utah-armlab'
    author = 'Michael Bentley (mikebentley15@gmail.com)'
    description = '''Motion planning and kinematic modeling for tendon-driven
        continuum robots.  This code was developed for the research project
        published under "Interactive-Rate Supervisory Control for
        Arbitrarily-Routed Multi-Tendon Robots via Motion Planning," in IEEE
        Access, 2022.'''
    license = 'BSD-2-Clause'
    topics = ('robotics', 'ros', 'motion-planning')
    homepage = 'https://sites.google.com/gcloud.utah.edu/armlab-tendon-planning'
    url = 'https://github.com/Utah-ARMLab/interactive-rate-tendons.git'
    requires = (
        'octomap/[>=1.0 <2]',
        'pybind11/[>=2.0 <3]',
        'fcl/[>=0.6 <0.7]',
        'libccd/[>=2.0 <3.0]',
        #'itk/[>=5.0.0 <6]',
        )
    generators = (
        'CMakeDeps',
        'CMakeToolchain',
        )
    settings = "os", "compiler", "build_type", "arch"
