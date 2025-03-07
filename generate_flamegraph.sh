OUTPUT_DIR="./flamegraph"

mkdir -p $OUTPUT_DIR

perf record -g "./out/build/GCC 11.4.0 x86_64-linux-gnu/VehicleExecutable"

# 导出 perf 数据
perf script > $OUTPUT_DIR/out.perf

# 使用 stackcollapse-perf.pl 脚本折叠数据
perl /home/xie/FlameGraph/stackcollapse-perf.pl $OUTPUT_DIR/out.perf > $OUTPUT_DIR/out.folded

# 生成火焰图
perl /home/xie/FlameGraph/flamegraph.pl $OUTPUT_DIR/out.folded > $OUTPUT_DIR/flamegraph.svg

echo "火焰图生成完成！文件保存在 $OUTPUT_DIR/flamegraph.svg"