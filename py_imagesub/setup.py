from setuptools import setup

package_name = 'py_imagesub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mfrizzell@thealbersgroup.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_imagesub.publisher_member_function:main',
            'listener = py_imagesub.subscriber_member_function:main',
            'streamer = py_imagesub.streamer_member_function:main',
        ],
    },
)
