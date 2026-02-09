import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


def analyze_traffic_data():
    try:
        # 1. 读取数据
        df = pd.read_csv('traffic_research_results.csv')

        if 'conflicts' not in df.columns:
            print("警告：CSV 文件中缺少 'conflicts' 列。请确保运行了最新版的仿真代码并重新导出。")
            # 临时补救：如果没有这一列，我们造一列全为 0 的数据防止脚本崩溃
            df['conflicts'] = 0

        # 设置绘图风格
        sns.set_theme(style="whitegrid")
        plt.rcParams['font.sans-serif'] = ['SimHei']  # 正常显示中文
        plt.rcParams['axes.unicode_minus'] = False

        # 计算不同驾驶类型的总冲突次数
        conflict_stats = df.groupby('type')['conflicts'].sum()
        print("\n--- 安全统计结果 (总冲突次数) ---")
        print(conflict_stats)

        # 2. 计算基本统计量
        stats = df.groupby('type')['wait_time'].agg(['mean', 'max', 'count']).reset_index()
        print("--- 仿真统计结果 ---")
        print(stats)

        # 3. 绘制对比柱状图
        plt.figure(figsize=(10, 6))
        ax = sns.barplot(x='type', y='wait_time', data=df, estimator='mean', palette='Set2')

        plt.title('不同驾驶行为下的平均排队等待时间对比', fontsize=15)
        plt.xlabel('驾驶员类型 (Aggressive vs Conservative)', fontsize=12)
        plt.ylabel('平均等待时间 (秒)', fontsize=12)

        # 在柱状图上方标注具体数值
        for p in ax.patches:
            ax.annotate(format(p.get_height(), '.2f'),
                        (p.get_x() + p.get_width() / 2., p.get_height()),
                        ha='center', va='center',
                        xytext=(0, 9),
                        textcoords='offset points')

        plt.figure(figsize=(10, 6))
        ax2 = sns.barplot(x=conflict_stats.index, y=conflict_stats.values, palette='OrRd')
        plt.title('不同驾驶行为下的安全冲突总数', fontsize=15)
        plt.ylabel('冲突次数 (距离 < 2m)', fontsize=12)
        plt.savefig('safety_comparison_chart.png', dpi=300)
        plt.show()

        # 4. 保存图表
        plt.savefig('comparison_chart.png', dpi=300)
        print("\n统计图表已保存为: comparison_chart.png")
        plt.show()

    except FileNotFoundError:
        print("错误：未找到 CSV 文件，请先运行仿真并按 'S' 键导出数据。")
    except Exception as e:
        print(f"发生错误: {e}")


if __name__ == "__main__":
    analyze_traffic_data()