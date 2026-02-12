import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob  # 用于匹配文件名


def analyze_traffic_data():
    # 1. 自动寻找 report 文件夹中最新的 traffic_analysis_*.csv 文件
    search_path = os.path.join('report', 'traffic_analysis_*.csv')
    files = glob.glob(search_path)

    if not files:
        print("❌ 错误：在 report 文件夹中未找到任何 traffic_analysis CSV 文件。")
        print("💡 提示：请确保你已经运行仿真并按过数字键 '8'。")
        return

    # 按文件修改时间排序，取最后生成的那个
    latest_file = max(files, key=os.path.getmtime)
    print(f"📊 正在分析最新数据文件: {latest_file}")

    # 2. 确定报告保存的名称（也可以带上对应的时间戳）
    # 获取文件名（不带扩展名），用于命名图片
    base_name = os.path.basename(latest_file).replace('.csv', '')
    output_image = os.path.join('report', f'report_{base_name}.png')
    flow_files = glob.glob(os.path.join('report', 'flow_data_*.csv'))
    if flow_files:
        latest_flow = max(flow_files, key=os.path.getmtime)
        df_flow = pd.read_csv(latest_flow)

        # 在 2x2 画布之外单独画一张全场效率图
        plt.figure(figsize=(10, 6))
        sns.lineplot(data=df_flow, x='Number_of_Vehicles', y='Avg_Speed_Efficiency')
        plt.title('系统宏观效率随车数变化趋势')
        plt.savefig(os.path.join('report', 'macro_efficiency.png'))
    try:
        df = pd.read_csv(latest_file)
        df = df.fillna(0)  # 填充空值

        # --- 绘图逻辑 (与之前一致) ---
        sns.set_theme(style="whitegrid")
        plt.rcParams['font.sans-serif'] = ['SimHei']
        plt.rcParams['axes.unicode_minus'] = False

        fig, axes = plt.subplots(2, 2, figsize=(15, 12))

        # 柱状图：平均等待时间
        sns.barplot(x='type', y='wait_time', data=df, ax=axes[0, 0], palette='coolwarm')
        axes[0, 0].set_title('平均排队等待时间 (秒)', fontsize=14)

        # 柱状图：总冲突次数
        conflict_data = df.groupby('type')['conflicts'].sum().reset_index()
        sns.barplot(x='type', y='conflicts', data=conflict_data, ax=axes[0, 1], palette='Reds')
        axes[0, 1].set_title('安全冲突总数', fontsize=14)

        # 箱线图：通行耗时
        completed_df = df[df['status'] == 'completed']
        if not completed_df.empty:
            sns.boxplot(x='type', y='travel_time', data=completed_df, ax=axes[1, 0], palette='Set2')
            axes[1, 0].set_title('车辆通行总耗时分布', fontsize=14)
        else:
            axes[1, 0].text(0.5, 0.5, '暂无已完成通行的车辆数据', ha='center')

        # 饼图：车型占比
        type_counts = df['type'].value_counts()
        axes[1, 1].pie(type_counts, labels=type_counts.index, autopct='%1.1f%%', colors=['#ff9999', '#66b3ff'])
        axes[1, 1].set_title('实验样本比例', fontsize=14)

        plt.tight_layout()
        plt.savefig(output_image, dpi=300)
        print(f"✅ 综合分析报告已保存至: {output_image}")
        plt.show()

    except Exception as e:
        print(f"❌ 分析出错: {e}")


if __name__ == "__main__":
    analyze_traffic_data()