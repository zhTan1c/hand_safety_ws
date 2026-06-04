from setuptools import find_packages, setup

package_name = 'hand_safety_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ketchup',
    maintainer_email='6251913079@stu.jiangnan.edu.cn',
    description='Safety layer for Inspire dexterous hand on real robot',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 'hand_safety_node = hand_safety_pkg.hand_safety_node:main',  # 已由 C++ 版替代
            'hand_safety_record_node = hand_safety_pkg.hand_safety_record_node:main',
        ],
    },
)
