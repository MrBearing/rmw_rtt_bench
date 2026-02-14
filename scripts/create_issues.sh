#!/usr/bin/env bash
# Create GitHub issues for identified code problems in rmw_rtt_bench.
# Usage: ./scripts/create_issues.sh
# Requires: gh CLI authenticated (run `gh auth login` first)

set -euo pipefail

REPO="MrBearing/rmw_rtt_bench"

echo "Creating issues for ${REPO}..."
echo ""

# ---- Issue 1 ----
gh issue create --repo "$REPO" \
  --title "LatencyStatsC 構造体がヘッダなしで重複定義されており ODR 違反のリスクがある" \
  --label "bug" \
  --body "$(cat <<'BODY'
## 概要

`LatencyStatsC` 構造体が `latency_stats.cpp` の `extern "C"` ブロック内で定義され、`latency_rtt_pinger.cpp` では独立して再宣言されています。共有ヘッダが存在しないため、一方を変更してもう一方を更新し忘れると、サイレントに不整合（ODR 違反）が発生します。

## 該当箇所

- `src/latency_stats.cpp:95-101` — 定義元
- `src/latency_rtt_pinger.cpp:19-20` — 重複宣言

## 再現方法

`latency_stats.cpp` の `LatencyStatsC` にフィールドを追加し、`latency_rtt_pinger.cpp` を更新しないままビルドすると、未定義動作になります。

## 提案

`LatencyStatsC` と関連する C インターフェース関数のプロトタイプを共有ヘッダ（例: `include/rmw_rtt_bench/latency_stats.h`）に移動する。
BODY
)"
echo "[1/10] Created: LatencyStatsC ODR violation"

# ---- Issue 2 ----
gh issue create --repo "$REPO" \
  --title "レガシー実行ファイルの Usage メッセージに誤ったパッケージ名が記載されている" \
  --label "bug" \
  --body "$(cat <<'BODY'
## 概要

`src/latency_rtt.cpp:238` の Usage メッセージに `ros2 run ros2_latency_bench ros2_latency_rtt` と記載されていますが、正しいパッケージ名は `rmw_rtt_bench` です。

## 該当箇所

- `src/latency_rtt.cpp:238`

```cpp
"Usage: ros2 run ros2_latency_bench ros2_latency_rtt -- ..."
```

## 期待される表記

```cpp
"Usage: ros2 run rmw_rtt_bench ros2_latency_rtt -- ..."
```
BODY
)"
echo "[2/10] Created: Wrong package name in usage message"

# ---- Issue 3 ----
gh issue create --repo "$REPO" \
  --title "rclcpp::shutdown() が複数回呼び出される可能性がある" \
  --label "bug" \
  --body "$(cat <<'BODY'
## 概要

全3つの実行ファイル（`rtt_pinger`, `rtt_ponger`, `ros2_latency_rtt`）で、コールバック内で `rclcpp::shutdown()` が呼ばれた後、`main()` 関数の末尾でも再び `rclcpp::shutdown()` が呼ばれます。

二重呼び出しは ROS 2 では通常安全ですが、タイミングによっては予期しない警告やエッジケースの原因になり得ます。

## 該当箇所

- `src/latency_rtt_pinger.cpp:190` (`finish_and_exit` 内) と `234` (`main` 末尾)
- `src/latency_rtt_ponger.cpp:59` (end_timer コールバック内) と `108` (`main` 末尾)
- `src/latency_rtt.cpp:120,128,133` (各コールバック内) と `257` (`main` 末尾)

## 提案

- コールバック内では `rclcpp::shutdown()` を呼ぶ代わりに `executor.cancel()` や フラグを使ってメインスレッド側で制御する
- または `main()` 側で `rclcpp::ok()` をチェックしてから呼ぶ
BODY
)"
echo "[3/10] Created: Double rclcpp::shutdown()"

# ---- Issue 4 ----
gh issue create --repo "$REPO" \
  --title "CSV フィールドがエスケープされておらず、特殊文字でデータが破損する" \
  --label "bug" \
  --body "$(cat <<'BODY'
## 概要

`transport_tag` と `notes` フィールドが CSV に書き出される際、エスケープ処理が行われていません。これらの値にカンマ（`,`）、改行、ダブルクォートが含まれると CSV フォーマットが破壊されます。

## 該当箇所

- `src/latency_rtt_pinger.cpp:142-152` — CSV 書き出し部分
- `src/latency_rtt.cpp:158-168` — 同上（レガシー版）

## 再現方法

```bash
ros2 run rmw_rtt_bench rtt_pinger -- --notes "hello,world" --duration 5
```

出力 CSV の `notes` カラムが分裂し、パーサーがエラーになります。

## 提案

- カンマや改行を含む文字列をダブルクォートで囲む（RFC 4180 準拠）
- または `transport_tag` / `notes` に使用できる文字を制限してバリデーションする
BODY
)"
echo "[4/10] Created: CSV field not escaped"

# ---- Issue 5 ----
gh issue create --repo "$REPO" \
  --title "rtt_zenoh_ponger.launch.py で RMW_IMPLEMENTATION が設定されていない" \
  --label "bug" \
  --body "$(cat <<'BODY'
## 概要

`launch/rtt_zenoh_ponger.launch.py:68` で `SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')` がコメントアウトされています。

他の Zenoh ランチファイル（`rtt_zenoh.launch.py`, `rtt_zenoh_pinger.launch.py`）では設定されているため、挙動に一貫性がありません。ponger 側で RMW が明示的に設定されないと、環境変数の設定漏れでマッチしない RMW が使われるリスクがあります。

## 該当箇所

- `launch/rtt_zenoh_ponger.launch.py:68`

```python
# SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'),
```

## 提案

意図的にコメントアウトされている場合は、その理由をコメントで記載する。そうでなければコメントアウトを解除する。
BODY
)"
echo "[5/10] Created: RMW_IMPLEMENTATION not set in ponger launch"

# ---- Issue 6 ----
gh issue create --repo "$REPO" \
  --title "シーケンス番号の型不一致による uint32 オーバーフローの可能性" \
  --label "bug" \
  --body "$(cat <<'BODY'
## 概要

内部カウンタ \`seq_\` は \`uint64_t\` ですが、メッセージフィールド \`msg.seq\` は \`uint32\` です。\`static_cast<uint32_t>(seq_++)\` により、\`seq_\` が 2^32 を超えると無警告でラップアラウンドし、CSV に記録されるシーケンス番号が重複します。

## 該当箇所

- \`src/latency_rtt_pinger.cpp:123\`: \`msg.seq = static_cast<uint32_t>(seq_++);\`
- \`src/latency_rtt.cpp:137\`: 同上
- \`msg/Rtt.msg:4\`: \`uint32 seq\`

## 影響

通常の使用では 100Hz で約497日間連続稼働しないと発生しないため、優先度は低いですが、高頻度（10kHz+）の長時間テストでは問題になり得ます。

## 提案

- \`msg.seq\` を \`uint64\` に変更する
- または \`seq_\` を \`uint32_t\` に統一して、ラップアラウンド時に警告ログを出す
BODY
)"
echo "[6/10] Created: Sequence number overflow"

# ---- Issue 7 ----
gh issue create --repo "$REPO" \
  --title "不明な CLI 引数が警告なく無視される" \
  --label "enhancement" \
  --body "$(cat <<'BODY'
## 概要

\`parse_pinger_args()\`, \`parse_ponger_args()\`, \`parse_rtt_args()\` の各関数で、認識できない引数（タイプミスを含む）が警告なく無視されます。

例えば \`--payload_size\`（ハイフンでなくアンダースコア）は、エラーにならず黙って無視されます。

## 該当箇所

- \`src/latency_rtt_pinger.cpp:44-64\` (\`parse_pinger_args\`)
- \`src/latency_rtt_ponger.cpp:25-42\` (\`parse_ponger_args\`)
- \`src/latency_rtt.cpp:39-69\` (\`parse_rtt_args\`)

## 提案

\`--\` で始まる認識できない引数に対して、警告ログを出力するか、エラーとして処理を中断する。

```cpp
// 例: 各パーサーの for ループ末尾に追加
else if (a.rfind("--", 0) == 0) {
  RCLCPP_WARN(rclcpp::get_logger("args"), "Unknown argument: %s", a.c_str());
}
```
BODY
)"
echo "[7/10] Created: Unknown CLI args silently ignored"

# ---- Issue 8 ----
gh issue create --repo "$REPO" \
  --title "run_payload_sweep.py が ponger のライフサイクルを管理しない" \
  --label "enhancement" \
  --body "$(cat <<'BODY'
## 概要

\`scripts/run_payload_sweep.py\` は pinger のみを起動し、ponger は別ターミナルで事前に手動起動する必要があります。しかし、スクリプトのヘルプメッセージやドキュメントにこの前提条件が記載されていません。

ponger が起動していない状態でスクリプトを実行すると、タイムアウトまで待機した後にエラーなく空の CSV が生成されます。

## 該当箇所

- \`scripts/run_payload_sweep.py\` 全体

## 提案

以下のいずれか（または両方）:
1. スクリプト内で ponger をサブプロセスとして自動起動・終了する
2. ヘルプメッセージに「ponger を事前に起動してください」と明記する
BODY
)"
echo "[8/10] Created: Payload sweep doesn't manage ponger"

# ---- Issue 9 ----
gh issue create --repo "$REPO" \
  --title "rtt_zenoh_pinger.launch.py にデッドコードと未使用 import がある" \
  --label "cleanup" \
  --body "$(cat <<'BODY'
## 概要

\`launch/rtt_zenoh_pinger.launch.py\` で Zenoh router 関連のコードがコメントアウトされていますが、関連する import 文（\`IfCondition\`, \`OpaqueFunction\`）がそのまま残っています。

## 該当箇所

- \`launch/rtt_zenoh_pinger.launch.py:3\`: \`from launch.conditions import IfCondition\`（未使用）
- \`launch/rtt_zenoh_pinger.launch.py:2\`: \`OpaqueFunction\`（未使用）
- \`launch/rtt_zenoh_pinger.launch.py:35-51\`: コメントアウトされた router コード
- \`launch/rtt_zenoh_pinger.launch.py:30-33\`: router 関連の変数宣言（\`start_router\`, \`router_listen\`, \`router_mode\`）が使用されない

## 提案

- コメントアウトされたコードと未使用の import を削除する
- router 機能が将来必要な場合は、\`rtt_zenoh_ponger.launch.py\` を参考に有効化するか、git 履歴から復元する
BODY
)"
echo "[9/10] Created: Dead code in pinger launch"

# ---- Issue 10 ----
gh issue create --repo "$REPO" \
  --title "自動テストスイートが存在しない" \
  --label "enhancement" \
  --body "$(cat <<'BODY'
## 概要

現在、ユニットテストや統合テストが一切存在しません。\`package.xml\` に \`ament_lint_auto\` / \`ament_lint_common\` のテスト依存は宣言されていますが、実際のテストコードはありません。

\`colcon test\` はリンティングのみ実行します。

## 影響

- コード変更時のリグレッション検出ができない
- 統計計算（\`latency_stats.cpp\`）の正確性が検証されていない
- CLI 引数パーサーのエッジケースが未テスト
- CSV 出力フォーマットの整合性が未検証

## 提案

以下のテストを優先的に追加する:

1. **latency_stats のユニットテスト**: 既知の入力に対して mean/median/p95/p99/stddev が正しいか検証
2. **CLI パーサーのテスト**: 正常系・異常系の引数パターン
3. **QoS ビルダーのテスト**: 有効/無効な QoS オプションの組み合わせ
4. **CSV フォーマットのテスト**: ヘッダ行とデータ行の整合性
BODY
)"
echo "[10/10] Created: No automated test suite"

echo ""
echo "All 10 issues created successfully."
