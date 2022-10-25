from setuptools import setup

package_name = 'sum_of_vectors'

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
    maintainer='gs',
    maintainer_email='gsrinivasan@wpi.edu',
    description='Summing two vectors',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = sum_of_vectors.service_member_function:main',
            'client = sum_of_vectors.client_member_function:main'
        ],
    },
)
