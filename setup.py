from setuptools import setup, find_packages

setup(
    name="RoundaboutTrafficSim",
    version="1.0.0",
    author="Your Name",
    description="基于 Pygame 的环岛交通流仿真与行为分析系统",
    long_description=open('README.md', encoding='utf-8').read() if os.path.exists('README.md') else "",
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/RoundaboutTraffic",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        'pygame',
        'pandas',
        'matplotlib',
        'seaborn',
        'numpy',
    ],
    entry_points={
        'console_scripts': [
            # 允许你在命令行直接输入 'run-sim' 来启动仿真
            'run-sim=simulation:main',
            # 允许你在命令行直接输入 'run-analysis' 来分析数据
            'run-analysis=analysis_report:analyze_traffic_data',
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.8',
)