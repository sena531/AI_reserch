# Buck-PFC 研究（長崎大学）

AC100Vrms 50Hz 入力の Buck-PFC（ブリッジレス PFC ＋ 位相シフトフルブリッジ）コンバータの
制御設計・シミュレーション・実機検証をまとめたリポジトリ。
制御器は dsPIC33EP128GS808（MPLAB X + XC16 v1.70）で実装する。

## フォルダ構成

```
buck-pfc-research/
├── simulation/
│   ├── scideam/          # Scideam 回路(.scicir) と制御スクリプト(.sciprc)
│   └── python/           # ACMC ゲイン設計・検証シミュレータと結果波形
├── firmware/
│   ├── BuckPFC.X/        # MPLAB X プロジェクト（ここを MPLAB で開く）
│   ├── src/              # 現行ソース（main_closed_loop.c ほか）
│   └── archive/          # 過去の制御コード（開ループ・DAB 試験・旧閉ループ版）
├── experiments/          # 実機試験結果・手順書・ノイズ対策・フォトカプラ解析
├── hardware/kicad/buck_pfc/  # 最新基板の回路図・PCB・Gerber（KiCad 10）
├── docs/                 # 制御設計メモ・スライド
└── .claude/skills/       # Claude Code 用スキル（電力変換器の制御設計）
```

## 主要ファイル

| 区分 | ファイル | 内容 |
|---|---|---|
| シミュレーション | `simulation/scideam/Buck PFC_closed.scicir` | Buck-PFC 閉ループ回路（最新） |
| | `simulation/scideam/closed_loop_buck_pfc.sciprc` | 上記の制御スクリプト（ファームの元） |
| | `simulation/scideam/bridgeless PFC.scicir` / `closed_loop_bridgeless_pfc.sciprc` | ブリッジレス PFC 単体の ACMC |
| | `simulation/scideam/PhaseShiftFullBridge*.sci*` | 位相シフトフルブリッジ単体 |
| | `simulation/python/pfc_acmc_sim.py` | 平均電流制御のゲイン設計・オフライン検証 |
| | `simulation/python/verify_sciprc_port.py` | .sciprc の逐語移植（転記ミス検出用） |
| 実機 | `firmware/src/main_closed_loop.c` | 現行の閉ループ制御（旧名 `closed_1103.c`） |
| | `experiments/kekka.xlsx` | 実機測定結果 |
| | `experiments/BuckPFC_閉ループ試験_障害解析.pptx` | 閉ループ試験の障害解析 |
| | `experiments/通電手順.html` / `実験前チェックリスト.html` | 実験手順 |
| | `experiments/ACライン_ノイズ対策手順.md` | AC ラインのノイズ対策 |
| | `experiments/photocoupler/` | Vo センシング（フォトカプラ）の実測解析 |
| 資料 | `docs/README_pfc_control.md` | ACMC 制御の設計メモ |

## 使い方

### MPLAB X
1. `firmware/BuckPFC.X` を MPLAB X で開く（デバイス: dsPIC33EP128GS808、コンパイラ: XC16 v1.70）。
2. ビルド対象は `src/main_closed_loop.c` と `src/traps.c`。過去版を試すときは
   `archive/` のファイルをプロジェクトに追加し、`main_closed_loop.c` を除外する。

### Python シミュレーション
```bash
cd simulation/python
python pfc_acmc_sim.py
```

### 文字コードについて
`firmware/` の `.c/.h` はローカルでは Shift-JIS（MPLAB 用）、GitHub 上では UTF-8 で保存される
（`.gitattributes` の `working-tree-encoding` で自動変換）。

## 含めていないもの
先行研究 PDF・修士論文・APEC 投稿資料・参考文献など（著作権・未公開情報のため）、
旧 KiCad プロジェクト、KiCad/MPLAB のバックアップとビルド生成物。
元データは OneDrive の `AI＿研究` フォルダに残っている。
