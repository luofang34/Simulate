---
name: vehicle-engineering-and-modeling
description: >-
  Reusable model-based pipeline for adding or refining a vehicle in the Simulate
  workspace: primary-source research with per-point provenance, package naming
  and layout, station-calibrated geometry, JSBSim flight model (powerplant,
  turbo/supercharger, propeller BEM maps, governors, mass), air-data and
  instrument-pack binding, closed-loop test flying against the source test card,
  and orthographic + GPU visual verification. Worked example: vehicles/p38g.
---

# `vehicles/` 載具工程考證與模型化流水線 (`skills.md`)

本文件是新增或精修 `Simulate` 載具的標準流水線，以 [`vehicles/p38g`](p38g/Cargo.toml)（Lockheed P-38G-13-LO）為完整範例，[`vehicles/c172`](c172/syspack.json) 為單發輕型機範例，[`vehicles/apollo-11`](apollo-11/Cargo.toml) 為非飛機範例。每個階段都有**輸入、產出物、驗收門檻**；後一階段只能建立在前一階段已通過門檻的產出物上。

| 階段 | 產出物 | 驗收門檻 |
|---|---|---|
| 1 原始資料考證 | 每個數據點附來源、重量、構型、功率設定與限制因素 | 無來源的數字不得進入 FDM 或試飛卡 |
| 2 命名與包結構 | `vehicles/<id>/` 目錄、crate、`syspack.json` | `cargo check`、`syspack_rehash.py`、builtin 載入測試 |
| 3 幾何與隔框放樣 | 網格、站位表、座艙預留 | 三視圖疊圖測試 |
| 4 飛行動力學 | `fdm/<id>.xml`、設計輸入 `design/*.json` | 引擎探針與試飛卡 |
| 5 大氣資料與儀表包 | 皮托靜壓裝置、PEC、儀表包引用 | PEC 載入與指示空速測試 |
| 6 閉環試飛 | `performance/poh_benchmarks.json` 試飛卡 | `ironbird --mode calibrate-dir` 全部 PASS |
| 7 視覺驗證 | 三視圖疊圖、GPU 多視角截圖 | 逐張目視檢查 |

---

## 1. 原始資料考證 (Stage 1)

先鎖定**具體批次／子型號**（P-38G-13-LO 而非泛稱 P-38；G 型無俯衝恢復襟翼、無液壓副翼助力、機翼前緣中冷器），不同批次的動力、氣動與系統不得混用。

### 1.1 文獻優先順序
1. **官方飛行試驗報告**（最高權重，因為含實測條件）：如 AAF Materiel Center Memorandum Report。P-38G 實際使用：
   - FS-M-19-1538-A（P-38G-1, 42-12687, Wright Field 1943-02-10）：3000 rpm 下各高度平飛速度、爬升率、進氣壓力、每發 BHP 與限制因素（化油器進氣溫度、渦輪轉速）。
   - FS-M-19-1492-A（P-38G, 42-12690, 1942-10-22）：失速速度，以及**皮托靜壓頭位置與位置誤差表**（Type D-2 Kollsman，左翼尖內側 87 in）。
   - 來源：`wwiiaircraftperformance.com/org/p-38/`（報告多為掃描 PDF，需逐頁讀圖）。
2. **飛行操作手冊 / T.O.**（限制速度、操作程序、控制面與檔位）。
3. **製造商規格書**（如 Lockheed Report 2338：保證性能、設計重量）。
4. **NACA 報告**（翼型、可壓縮性）。
5. **遊戲或模擬器社群資料**：只能用作交叉提示，不得作為數值來源。

### 1.2 每個數據點必須記錄
重量、重心、構型（襟翼／起落架／冷卻風門）、高度、功率設定（進氣壓力 inHg + rpm，而非「全油門」）、混合比、**限制因素**。例：42-12687 在 22,600 ft 以 44.7 inHg 達 404 mph，該高度即渦輪臨界高度；24,400 ft 起受化油器進氣溫度限制；30,000 ft 起受渦輪轉速限制。這些限制因素直接決定 FDM 的增壓模型（§4.2）。

### 1.3 IAS / 水柱空速 / CAS 的拆分
試驗報告的空速表常分四欄：指示空速 (IAS)、水柱空速（扣除儀表誤差，即真實皮托靜壓差）、校正空速 (CAS)、裝機誤差。**儀表誤差屬於儀表本身（進儀表包），裝機（位置）誤差屬於機體靜壓孔位置（進 PEC）**。42-12690：IAS 115 → 水柱 113 → CAS 127 mph，即儀表誤差 +2、位置誤差 −14 mph。

### 1.4 座標系
| 座標系 | +X | +Y | +Z | 單位 | 用途 |
|---|---|---|---|---|---|
| 工廠站位 `FS`/`BS`/`WS`/`WL` | `WS` 右翼展向 | `FS`/`BS` 向後 | `WL` 向上 | in | 藍圖、隔框表 |
| 建模 / OpenUSD | 右 | 前 | 上 | m | `vehicles/<id>` 網格與 `.usda` |
| `syspack.json` 機體 | 前 | 右 | 下 | m | 質量、裝置（含皮托靜壓）掛載點 |
| JSBSim 結構 | 後 | 右 | 上 | in | `<metrics>`、`<mass_balance>`、推進器位置 |

---

## 2. 命名與包結構 (Stage 2)

### 2.1 命名
- 目錄、套件 id、crate 名一律用**小寫機型代號＋子型號**：`vehicles/p38g`、`"id": "p38g"`、`sim-p38g`、`fdm/p38g.xml`。只有在子型號會造成氣動或系統差異時才帶子型號（P-38G 與 J/L 的動力、冷卻與副翼助力不同，故為 `p38g`；未來 J 型為獨立 `p38j`）。
- **單一機身（序號、部隊、機名如 "Miss Virginia"）是塗裝，不是載具**：只出現在 `Liveries/catalog.json`，不得寫進 crate 名稱或描述。
- 改名時需同步：目錄、crate 名、workspace member 與依賴別名、`syspack.json` 的 `id`/`name`、JSBSim `fdm_config name` 與 `dynamics.JsbsimXml`、塗裝目錄的 `aircraft_id`/`dlc_mount_point`、`sim-pack/src/builtin.rs` 的 `include_bytes!` 路徑、`simulate-app`/`sim-maplibre` 的符號。最後以 `grep` 舊字串歸零、並 `cargo check` 所有下游 crate。

### 2.2 依賴規則
渲染器解耦：載具 crate **不得依賴** `sim-maplibre`、`sim-render`、`wgpu` 或視窗系統；渲染適配層只消費載具 crate 產出的網格與動畫狀態。可依賴數學、序列化、`openusd`、`mbd-core`、`sim-core`、`sim-pack`、`sim-geo`、`sim-atmosphere`（`apollo-11` 與 `raketa` 皆依賴部分 `sim-*` crate）。CI 尚未強制此規則，提交前自行檢查 `Cargo.toml`。

### 2.3 目錄
```text
vehicles/<id>/
├── Cargo.toml / src/          # 幾何 crate（如有）
├── syspack.json               # 信任根：每個資產的 sha256 與大小
├── fdm/<id>.xml               # JSBSim 模型
├── design/*.json              # 生成表格的設計輸入（如螺旋槳規格）
├── performance/
│   ├── poh_benchmarks.json        # 試飛卡（§6）
│   ├── airspeed_envelope.json     # V 速度（PFD/HUD 標記）
│   └── pitot_static_calibration.json  # 位置誤差（§5）
└── Liveries/catalog.json + Models/Liveries/*.xml
vehicles/tools/
├── propeller_bem.py           # 螺旋槳 BEM 表格與調速器反演表
├── part_jsbsim.py             # 由零件的 engine.json / propeller.json 生成 JSBSim 片段與 ace-rs 目錄條目
└── syspack_rehash.py          # 修改資產後刷新 sha256/size 與儀表包、零件的 pin
parts/
├── engines/<id>/              # kind: Engine：engine.json（逐值出處）＋ jsbsim/engine.xml、jsbsim/system-<n>.xml、ace/catalog-entry.json
└── propellers/<id>/           # kind: Propeller：propeller.json ＋ jsbsim/propeller.xml、jsbsim/system-<n>.xml、ace/catalog-entry.json
```

### 2.4 可重用零件（發動機、螺旋槳）
發動機與螺旋槳是獨立、有版本、以 sha256 釘住的零件包（與儀表包同形）。載具 manifest 以 `parts: [{role, id, version, manifest_sha256}]` 釘住，`role` 為 `engine0`、`propeller1` 等；`sim-pack` 依 role 將零件的 JSBSim 文件放在 `engine/<role>.xml`（`<engine file="engine0">`、`<thruster file="propeller0">`）與 `systems/<role>.xml`（`<system file="engine0"/>`）。JSBSim 系統無法參數化 `propulsion/engine[n]`，故零件為每個發動機位置各帶一份 `jsbsim/system-<n>.xml`，由 role 結尾的數字選用；發動機系統須排在螺旋槳系統之前（調速器讀取發動機系統公佈的 `available-power-hp`）。修改 `engine.json`/`propeller.json` 後依序執行 `part_jsbsim.py`、零件的 rehash，再 rehash 釘住它的載具。

同一機體的不同動力構型是**變體**，不是複製的 FDM：機體目錄只有一份 `fdm/<id>.xml`（以 `file=` 引用零件），預設變體為 `syspack.json`，其他變體為 `variants/<v>.json`（`DirectoryPackageSource::with_manifest`；rehash 時加上 manifest 路徑）。各變體只宣告自己用到的共用資產，並各帶 `performance/<v>/poh_benchmarks.json` 試飛卡（宣告兩張卡即拒絕載入）。範例：`vehicles/g4m1`（火星 11 型為校準基準，火星 15 型為同機體換發動機零件）；單一機身預設掛哪個變體記在 `design/airframes.json`，可隨證據修改。
`syspack.json` 為 fail-closed：**任何資產修改後必須執行** `python3 vehicles/tools/syspack_rehash.py vehicles/<id>`，否則載入即失敗。新增資產時先在 `assets` 加入條目，再 rehash；若為 builtin 載具，同步在 `sim-pack/src/builtin.rs` 以 `include_bytes!` 掛上。

`./scripts/check-structure.sh` 規則：禁止 `mod.rs`/`utils.rs`/`helpers.rs`/`common.rs`；每檔 ≤ 500 行、`lib.rs` ≤ 100 行並以 `//!` 開頭、每函式 ≤ 80 行；非測試碼不得 `+= 1`。

---

## 3. 幾何與隔框放樣 (Stage 3)

（以 `vehicles/p38g/src` 為範例）
- `stations.rs`：三視圖基準尺寸與 `FS`/`BS`/`WS`/`WL` 隔框常數表；隔框間以 smoothstep 插值、超橢圓截面連接。
- 主翼以真實 NACA 四／五位數厚度與彎度線放樣（P-38G：翼根 23016 → 翼尖 4412），襟翼與副翼區段預留鉸鏈槽並給出沿鉸鏈線的旋轉軸。
- 座艙蓋分片（防彈平板風擋、側窗、頂蓋、後窗），玻璃低粗糙度、半透明。
- 動力艙：減速器殼、排氣管、渦輪增壓器、中冷器與散熱器風門依批次區分。
- 螺旋槳三態（靜止實體槳葉、高速槳盤、順槳 87°），順槳旋轉軸為槳葉展向軸。
- 塗裝貼花沿蒙皮法線偏移 0.6–0.8 mm，弧面圖案做重心座標細分（N ≥ 5）。
- `cockpit_reservation.rs` 公開 `CockpitInteriorContract`：隔框、儀表板框（`PanelMountFrame`）與儀表槽位、操縱桿、油門台、座椅。**儀表槽位只定義面板座標框；裝什麼儀表由儀表包決定（§5.3）**。

---

## 4. 飛行動力學 (Stage 4)

### 4.1 質量
`emptywt` 用基本空重（含固定武器），其餘以 pointmass 分列：`pointmass[0]` 為**酬載槽**（飛行員），試飛工具會調整它來達到試飛卡的總重；彈藥、滑油各自一個 pointmass。燃油按真實油箱分列並指定 `<feed>`（P-38G：每側 90 gal 主油箱＋60 gal 備用油箱，左發供油 0/2、右發 1/3）。

### 4.2 渦輪增壓活塞發動機
JSBSim piston 的增壓器以「每檔一個固定壓比 × 油門」建模，並以 `ratedboost` 作為調壓器上限。正確作法：
1. **額定進氣壓力**：`ratedboost = MAP_rated − 29.92 inHg`；起飛／戰鬥壓力以 `takeoffboost` 表示（油門 > 0.98 才開放）。
2. **以多檔增壓代替連續廢氣門**：單一大壓比會使低空油門失去作用（P-38 低空怠速仍有 ~40 inHg）。用 3 檔、依高度自動切換（P-38G：6,000 / 15,000 / 24,700 ft），使低空油門可從怠速連續調到額定。
3. **最高檔額定高度**由試驗中臨界高度以上的進氣壓力衰減反推（P-38G：使 30,000 ft 得 36 inHg），而非直接填臨界高度；`ram-air-factor` 調低（0.2），因為臨界高度以上受渦輪轉速限制而非衝壓。
4. **增壓充氣加熱**：JSBSim 以環境溫度計算進氣密度。在 FCS 內計算壓氣機出口溫度（η_c 0.65）與中冷器效率（P-38G 0.5），以 `T_amb/T_charge` 縮放 `volumetric-efficiency`；同一溫度即為化油器進氣溫度表的讀數（P-38G 限制 57 °C）。少了這一步，相同進氣壓力下的功率會隨高度上升 30% 以上。
5. **自動混合比**（壓力化油器）：JSBSim 的當量比為 `1.3 × mixture × p0/p_amb`；在 FCS 中依手柄檔位（關斷／自動貧油／自動富油）寫入 `mixture-pos = φ/1.3 × p_amb/p0`。
6. 最後只用 `bsfc`（實為指示比油耗）縮放絕對功率，對準試驗的每發 BHP。

### 4.3 螺旋槳
- 以 `vehicles/tools/propeller_bem.py` 從 `design/<prop>.json`（槳葉數、直徑、弦長與厚度分佈、設計進距比）生成 `C_THRUST`/`C_POWER(J, β_0.75R)` 二維表；`eta` 輸出用於檢查最高效率（WWII 金屬槳約 0.82–0.86）。
- **定距槳**（C172）：BEM 在低進距比（J < 0.3，起飛滑跑）不可靠：對照 NACA TR 640 的 5868-9 實測，靜態 Cp 低 35–55%（`vehicles/tools/propeller_check.py` 有測試）。因此以 `vehicles/tools/propeller_measured.py` 生成表格：J < 0.3 用 TR 640 實測表並加 TR 684 Fig. 18 的槳尖馬赫功率損失（TR 640 在槳尖 M≈0.5 測得，C172 靜態約 0.7），J ≥ 0.4 用 BEM（`bem_above`），中間線性過渡。發動機先以兩點校正（Lycoming 全油門曲線：2,300 與 2,700 rpm，有測試）；滑跑時槳後被吹的機身零件另加滑流阻力（TR 640 p. 23）；試飛工具的起飛技術依 POH（機尾略低、在 VLOF 離地，有 ±1 kt 測試）。無法同時滿足的列、檢查與表格台階記在 `performance/validation.json`，測試逐項比對，而非加無來源的修正係數。
- 試驗報告的槳葉角若在其他半徑（42-12687：42 in 處 22.7–57.7°），依扭轉換算到 0.75R（16.8–51.8°）。
- 槳尖馬赫損失用 `CT_MACH`/`CP_MACH`；P-38G 在高空 400 mph 時槳尖馬赫約 1.05–1.1，效率約損失一成。
- **定速調速器放在 FCS，而不是只靠 JSBSim 內建**：內建的 `pitch -= ΔRPM·dt` 在配平的 0.5 s 穩態迭代中振盪。做法：螺旋槳保留 `minrpm`/`maxrpm`（內建調速器只在「發動機運轉中」生成時的穩態求解裡起作用，那裡不執行 FCS；少了它槳葉停在細距、靜止推力數千磅），FCS 以 `propeller_bem.py ... beta` 生成的反演表 β(J, Cp) 前饋求平衡槳角（前饋功率取軸功率與「進氣壓力可提供功率」較大者，避免風車起動卡在細距）、加慢速 RPM 積分修正，輸出到 `propulsion/engine[n]/blade-angle`。**必須公開 `propulsion/engine[n]/governor-rpm-req`**，試飛工具以此判斷該機為定速槳。
- 順槳由 FCS 積分器驅動槳角到 87°（電動槳約 5 s，且發動機停轉時仍可順槳／回槳）；風車阻力與不對稱偏航由螺旋槳模型自然產生，不再另寫阻力表。

### 4.4 氣動
- 失速 CLmax 依試驗失速 CAS（非 IAS）與重量反推；襟翼增量分開調。
- 冷卻風門阻力只計算**超出試驗時中立位置**的增量（基本極曲線已含中立位置的冷卻阻力）。
- 無俯衝恢復襟翼機型需 Mach tuck（低頭力矩）與尾翼效能衰減；無助力副翼依動壓衰減滾轉權限。

---

## 5. 大氣資料與儀表包 (Stage 5)

### 5.1 分層
| 層 | 位置 | 職責 |
|---|---|---|
| 物理與機構模型 | `~/mbd` 的 `mbd-airdata`（`no_std`） | ISA、可壓縮皮托關係（qc ↔ CAS/Mach）、管路延遲、靜壓位置誤差、各儀表機構（膜盒空速表、膜盒＋Kollsman 高度表、毛細管洩漏式升降速度表、壓力表、轉速表）與指針二階動態 |
| 契約與信任 | `~/mbd` 的 `syspack` | `PitotStatic` 裝置（皮托與靜壓孔的機體座標、探頭型式、管路時間常數）、`instrument_pack` 引用（id + 版本 + 儀表包 manifest 的 sha256）、`kind: InstrumentPack` 儀表包與其 `instruments.json` 驗證 |
| 載具 | `vehicles/<id>` | 皮托靜壓裝置位置、`pitot_static_calibration.json`、引用哪個儀表包、座艙面板座標框 |
| 共用儀表包 | 每個包一個 syspack（`kind: InstrumentPack`） | 儀表清單（真實型號、量程、非線性刻度、色弧、機構常數、儀表誤差）、預設面板佈局；多架飛機共用同一包（C172 與其他現代 GA 共用一包；P-38G 用 1943 USAAF 戰鬥機包） |
| 顯示 | `~/Indicate`（玻璃座艙 PFD/HSI）／3D 機械儀表幾何 | 只消費狀態（指針角、讀數），不計算物理 |
| 設計 | `~/ace-rs` | 新設計時由靜壓孔位置估算 PEC、選擇儀表包，並輸出上述資產 |

### 5.2 位置誤差 (PEC)
`performance/pitot_static_calibration.json` 每條襟翼曲線為 `{kias, kcas}` 點，其中 `kias` 應為**水柱空速**（只含位置誤差），儀表誤差留給儀表包。試驗範圍外的點必須在 `reference_document` 中明確標示為假設（P-38G：誤差在 200 mph IAS 線性收斂到零）。

### 5.3 儀表包（`instruments/<pack-id>/`）
儀表包本身是一個 syspack：`kind: "InstrumentPack"`、`dynamics: "None"`、`devices: []`、一個 `panel` 元件（儀表殼質量），以及資產 `instruments.json`（schema 在 `syspack::instrument_pack`，`deny_unknown_fields`）：
```json
{ "instruments": [
    { "id": "airspeed", "name": "AAF airspeed indicator, 0-700 mph",
      "bezel": "ThreeAndOneEighthInch", "source": "pitot_static.pitot_static",
      "needle": { "natural_frequency_rad_s": 12.0, "damping_ratio": 0.7 },
      "markings": [ { "Radial": { "at": 420.0, "colour": "Red" } } ],
      "kind": { "Airspeed": { "units": "MilesPerHour",
                "scale": [ { "value": 0, "angle_deg": 0 }, { "value": 700, "angle_deg": 345 } ],
                "instrument_error": [ [95.0, 2.0], [115.0, 2.0] ] } } } ],
  "layout": [ { "slot": "flight_left", "instrument": "airspeed", "offset_m": [-0.10, 0.05] } ] }
```
- `kind`：`Airspeed`（膜盒，依 CAS 定律刻度，可含 `instrument_error`）、`Altimeter`（Kollsman 視窗範圍、膜盒時間常數、量程）、`VerticalSpeed`（毛細管洩漏時間常數）、`PressureGauge`（進氣壓力、滑油壓力等）、`Tachometer`。刻度為單調的非線性 `scale`；色弧／色線用 `markings`。
- `source` 綁定語法封閉，打錯即載入失敗：`pitot_static.<裝置 id>`、`engine[n].<量>`（如 `engine[0].manifold_pressure`、`engine[1].rpm`）、`system.<路徑>`。
- `name` 只寫能查證的型號（如 Kollsman Type C-12）；查不到時寫功能與量程，不要編造型號。
- 目前內建兩個包：`ga-analog-six-pack`（C172）、`usaaf-1943-fighter`（P-38G：空速、C-12 高度表、升降速度、雙進氣壓力、雙轉速）。

### 5.4 載具端綁定
- `syspack.json` 的 `devices` 加 `PitotStatic`（`pitot_body_m`、`static_body_m` 為機體前-右-下座標，`probe` 描述安裝型式，`total_line_tau_s`/`static_line_tau_s` 為管路時間常數）。P-38G：Type D-2 Kollsman 頭在左翼尖內側 87 in，靜壓孔在前緣後 28 in、下翼面下 14-1/4 in。
- 頂層 `instrument_pack: { "id", "version", "manifest_sha256" }` 釘住儀表包。`sim-pack` 的 `LoadedSyspack::load` 會以內建包解析並驗證此 pin，且檢查每個 `pitot_static.<id>` 都對應到本機的 `PitotStatic` 裝置；`LoadedSyspack::instrument_pack()`、`pitot_static_device(id)` 提供給顯示層。
- 修改包或載具後依序 rehash：先 `instruments/<pack>`，再引用它的載具（`syspack_rehash.py` 會同時刷新 pin）。
- 驗收：`crates/sim-pack/src/instruments/tests.rs` 以 P-38G 的 PEC、皮托靜壓壓力與包內空速表機構，在 127 mph CAS 重現 42-12690 報告的 115 mph 指示。新機型需加入同類「試驗報告一列 → 指示讀數」的測試。

---

## 6. 閉環試飛 (Stage 6)

### 6.1 試飛卡 `performance/poh_benchmarks.json`
- `stalls`：`target_kcas`（試驗 CAS）、`target_kias`（水柱 IAS，與 PEC 一致）、重量、襟翼。
- `climb_points`：高度、重量、`climb_kcas`、`target_fpm`，增壓機加 `target_map_inhg`（試驗保持的進氣壓力），額定轉速低於調速器上限時加 `target_rpm`。
- `climb_time_points`：試驗只給爬升時間時用；起訖高度、重量、空速排程（`climb_kcas` + `climb_kcas_per_1000ft`）、進氣壓力與轉速、`target_min`，可選 `from_brake_release`（由放剎車起算，先飛起飛段）。以每 1,500 ft 一階的穩定爬升對能量高度積分。
- `cruise_points`：高度、重量、`target_rpm`（座艙轉速表讀數，即曲軸轉速）、`target_ktas`，增壓機加 `target_map_inhg`。
- `rated_bhp` 為全機總額定功率；`reference_document` 寫明報告編號與條件。

### 6.2 試飛工具的行為（`sim-pack` calibration）
- 每個有動力的試驗點**使用新載入的模型**（等同分開的試飛架次），不帶入前一點的發動機、調速器或燃油狀態。
- 所有油門、混合比、螺旋槳手柄寫入**每一台發動機**（`fcs/*-cmd-norm[n]`）；JSBSim 不帶索引的名稱只作用於 0 號發動機。
- 定速槳（JSBSim `constant-speed-mode` 或 FCS `governor-rpm-req`）以螺旋槳手柄保持 `target_rpm`、以油門保持 `target_map_inhg`；定距槳以油門保持 rpm。
- 爬升：60 s 飛行、最後 15 s 量測，量測**能量高度** `h + V²/2g` 的變化率；速度環寬鬆、姿態環高阻尼，避免重型機在低密度下的速度／姿態振盪吃掉爬升率。
- 平飛：以 6 s 為一段持續飛行，直到相鄰兩段平均真空速相差 < 0.05 kt（至少 30 s、最多 600 s），取最後一段；固定飛行時間會讓重型機仍在加速，結果取決於起始速度（`tests/level_speed.rs`）。

### 6.3 指令
```sh
# 全卡（Markdown 報表）
cargo run -p ironbird -- --mode calibrate-dir --package-dir vehicles/<id>
# 回歸測試（P-38G）
cargo test -p sim-pack --test p38g_flight_test
# 發動機工作點探針（調參用）
cargo test -p sim-pack --test p38g_flight_test engine_operating_points -- --ignored --nocapture
```
調參順序：先用探針對準每發功率與進氣壓力（§4.2），再看平飛速度（阻力、螺旋槳效率），最後看爬升與失速。每次修改 FDM 後先 rehash。

### 6.4 P-38G 目前結果（作為新機型的參考精度）
失速 CAS 誤差 < 1 kt；爬升 SL–30,000 ft 誤差 −113…+272 fpm；平飛 5,000–30,000 ft 五點誤差 ≤ 5.7 kt。

---

## 7. 視覺驗證 (Stage 7)

1. **與工程圖比對**（純 Rust 光柵化）：網格的側、俯、前視輪廓對數位化的 AN 01-75F-2 三視圖（`vehicles/p38g/design/an01-75f-2_general_arrangement.json`，英寸，附像素出處）計算 IoU 與邊界偏差；設定輸出目錄時寫出差異圖 `p38g_silhouette_<view>.png`（紅：只有圖紙，藍：只有網格）：
   ```sh
   SIMULATE_VERIFY_OUT=/path/to/out cargo test --release -p sim-p38g mesh_silhouettes_match_the_an01_75f_2_general_arrangement
   ```
   與原圖逐像素對齊的正射疊圖（圖紙本身不入庫，疊圖需另外合成到原圖上）：
   ```sh
   SIMULATE_VERIFY_OUT=/path/to/out cargo test --release -p simulate-app render_fidelity_audit_ortho_overlays -- --ignored
   ```
2. **GPU 多視角**（機庫 3/4、跑道追尾、前側方、正側、空中大坡度、單發順槳）：
   ```sh
   SIMULATE_VERIFY_OUT=/path/to/out cargo test -p simulate-app --bin simulate-app render_p38_third_person_verification_shots -- --ignored --nocapture
   ```
   逐張以 Read 工具目視檢查。

---

## 8. 提交前門檻與已知陷阱

```sh
cargo fmt --all --check
./scripts/check-structure.sh
./scripts/check-architecture.sh
cargo clippy -p <vehicle-crate> -p sim-pack -p sim-maplibre -p simulate-app --all-targets -- -D warnings
cargo test -p <vehicle-crate> -p sim-pack
```
完整 CI 為 `./scripts/ci.sh`。

陷阱：
- **jsbsim-rs 活塞發動機在同一模型第二次 `run_ic` 後不會重新運轉**（`propulsion/set-running` 不生效）；試飛工具因此每點重新載入模型。
- **生成時的槳距**：`EngineStartMode::Running` 會跑 JSBSim 的發動機穩態求解，不執行 FCS；FCS 調速器的機型必須同時保留內建調速器，否則停機坪上以 3,300 rpm 細距起步並自行滑走。
- **JSBSim 配平**（`do_trim(Full)`）在 FCS 驅動槳角的機型上不收斂（槳角由 FCS 給出，配平的穩態推進迭代不迭代 FCS），且增壓調壓器的平坦區使油門軸導數為零。配平失敗不會破壞狀態；需要穩定飛行的測試改用短時間閉環穩定。
- **JSBSim 把全富油（φ 1.3）當作最佳功率混合比**；真實發動機全富油較最佳功率更富、功率略低。C172 在海平面與 2,000 ft 全富油爬升因此高出 POH 45–65 fpm。
- 這台機器的 `cp` 是互動式別名；腳本中覆寫檔案用 `\cp -f`。
- 共用 `CARGO_TARGET_DIR`，磁碟接近滿載時先 `df -h ~` 再建置。
