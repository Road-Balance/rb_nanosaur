from setuptools import setup

package_name = 'webrtc_pkg'
submodule_rtccam = "webrtc_pkg/RTCCam"
submodule_rtcpub = "webrtc_pkg/RTCPub"


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_rtccam, submodule_rtcpub],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtc_client_node   = webrtc_pkg.rtc_client:main',
        ],
    },
)
