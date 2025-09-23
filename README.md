# rmw_rtt_bench

ROS 2 latency benchmark suite with two modes:
- One-way pub→sub（時刻同期推奨）
- RTT ping–pong（時計同期不要、RTTから片道近似も算出）

CSVは既定で`results/`配下に保存します。QoSとRMWは実行時に切替可能です。

## Build

```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-select rmw_rtt_bench --symlink-install
source install/setup.bash
```

【一方向モードは本パッケージから削除しました。RTTモードをご利用ください。】

## RTT Ping–Pong モード

実行ファイル: `rtt_pinger` / `rtt_ponger`

- 同一PC（Fast DDS例）
  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 run rmw_rtt_bench rtt_ponger -- --duration 65
  ros2 run rmw_rtt_bench rtt_pinger -- \
    --hz 100 --payload-size 1024 --duration 60 --csv results/rtt.csv \
    --qos-reliability reliable --qos-history keep_last --qos-depth 10 --append-summary true
  ```
- zenoh + Launch（同時起動、ルータ自動起動可）
  ```bash
  ros2 launch rmw_rtt_bench rtt_zenoh.launch.py \
    start_router:=true domain_id:=42 router_listen:=tcp/0.0.0.0:7447 router_mode:=router \
    hz:=100 payload_size:=1024 duration_pinger:=60 duration_ponger:=65 \
    qos_reliability:=reliable qos_history:=keep_last qos_depth:=10 csv:=results/rtt.csv
  ```
  別PCで個別起動: `rtt_zenoh_pinger.launch.py` / `rtt_zenoh_ponger.launch.py`（`start_router`でrmw_zenohd起動ON/OFF）。

RTT CSVヘッダ: `seq,t0_ns,t1_ns,t2_ns,t3_ns,rtt_ns,proc_ns,oneway_est_ns,payload_size,rmw,qos_rel,qos_hist,qos_depth,transport_tag,host,notes`

## Notes
- RMW切替: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp|rmw_cyclonedds_cpp|rmw_zenoh_cpp` など。
- 遠隔の一方向計測はNTP/PTP等での高精度同期が必須。同期不要な計測はRTTモードを使用。
- 解析補助（任意）: `python3 src/rmw_rtt_bench/scripts/summarize_rtt.py results/rtt.csv --append`
