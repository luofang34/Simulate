---
name: vehicle-engineering-and-modeling
description: >-
  Reusable model-based pipeline for adding or refining a historically accurate
  vehicle in the Simulate workspace: batch/serial lock and source verification
  with per-value provenance, package naming and layout, drawing digitisation and
  drawing-frame geometry, JSBSim flight model calibrated sourced-physics-first
  (powerplant, measured and BEM propeller maps, governors, mass), air-data,
  instrument packs and cockpit, closed-loop test flying against the source test
  card with a validation record, per-vehicle camera views, and photo alignment
  by camera resection for exterior and cockpit. Worked examples: vehicles/p38g,
  vehicles/c172s, vehicles/c172p.
---

# `vehicles/` 載具工程考證與模型化流水線 (`skills.md`)

本文件是新增或精修 `Simulate` 載具的標準流水線，以 [`vehicles/p38g`](p38g/Cargo.toml)（Lockheed P-38G-13-LO）為完整範例，[`vehicles/c172s`](c172s/syspack.json)（N642SP，172S8050）與 [`vehicles/c172p`](c172p/syspack.json)（D-ECJB，Reims F172P）為單發輕型機子型號與尾號塗裝分離範例，[`vehicles/apollo-11`](apollo-11/Cargo.toml) 為非飛機範例。每個階段都有**輸入、產出物、驗收門檻**；後一階段只能建立在前一階段已通過門檻的產出物上。

| 階段 | 產出物 | 驗收門檻 |
|---|---|---|
| 1 原始資料考證 | 每個數值附來源、頁、構型、功率設定與限制因素 | 無來源的數字不得進入 FDM 或試飛卡；轉錄表逐頁核對 |
| 2 命名與包結構 | `vehicles/<id>/` 目錄、crate、`syspack.json` | `cargo check`、`syspack_rehash.py`、builtin 載入測試 |
| 3 幾何 | 數位化工程圖、網格、`geometry/landmarks.json` | 輪廓與尺寸標註測試、結構基準殘差、輪底對 FDM 接地點 |
| 4 飛行動力學 | `fdm/<id>.xml`、設計輸入 `design/*.json` | 引擎探針、螺旋槳表可由設計檔重生 |
| 5 大氣資料、儀表與座艙 | 皮托靜壓裝置、PEC、儀表包引用、座艙契約 | PEC 載入與指示空速測試 |
| 6 閉環試飛 | `performance/poh_benchmarks.json`、`performance/validation.json` | 試飛卡逐列結果與驗證紀錄一致 |
| 7 視覺 | 表面、`views/cameras.json`、照片對齊 | 對齊視角下逐張目視、重投影殘差測試 |

---

## 1. 原始資料考證 (Stage 1)

先鎖定**具體批次／子型號或序號**（P-38G-13-LO 而非泛稱 P-38；G 型無俯衝恢復襟翼、無液壓副翼助力、機翼前緣中冷器），不同批次的動力、氣動與系統不得混用。

### 1.1 文獻優先順序
1. **官方飛行試驗報告**（含實測條件，權重最高）：如 AAF Materiel Center Memorandum Report FS-M-19-1538-A（P-38G-1 42-12687：各高度平飛速度、爬升率、進氣壓力、每發 BHP 與限制因素）、FS-M-19-1492-A（42-12690：失速速度、皮托靜壓頭位置與位置誤差表）。掃描 PDF 需逐頁讀圖。
2. **飛行操作手冊 / T.O. / POH**（限制速度、程序、性能表）。
3. **製造商規格書**（保證性能、設計重量）與**工程圖**（總體佈置、站位）。
4. **NACA 報告**（翼型、螺旋槳實測、可壓縮性）。
5. **遊戲或模擬器社群資料**：只能作交叉提示，不得作為數值來源。

### 1.2 每個數據點必須記錄
重量、重心、構型（襟翼／起落架／冷卻風門）、高度、功率設定（進氣壓力 inHg + rpm，而非「全油門」）、混合比、**限制因素**。例：42-12687 在 22,600 ft 以 44.7 inHg 達 404 mph，該高度即渦輪臨界高度；24,400 ft 起受化油器進氣溫度限制；30,000 ft 起受渦輪轉速限制。限制因素直接決定增壓模型（§4.2）。

### 1.3 IAS / 水柱空速 / CAS 的拆分
試驗報告的空速常分四欄：指示空速、水柱空速（扣除儀表誤差）、校正空速、裝機誤差。**儀表誤差屬於儀表（進儀表包），位置誤差屬於機體靜壓孔（進 PEC）**。42-12690：IAS 115 → 水柱 113 → CAS 127 mph，即儀表誤差 +2、位置誤差 −14 mph。

### 1.4 座標系
| 座標系 | +X | +Y | +Z | 單位 | 用途 |
|---|---|---|---|---|---|
| 工廠站位 `FS`/`BL`/`WL` | `FS` 向後 | `BL` 向右 | `WL` 向上 | in | 工程圖、隔框表、JSBSim 結構框架（§3） |
| 建模 / OpenUSD | 右 | 前 | 上 | m | `vehicles/<id>` 網格與 `.usda` |
| `syspack.json` 機體、`views/cameras.json` | 前 | 右 | 下 | m | 質量、裝置掛載點、相機位置 |

### 1.5 來源核對
- **手冊適用性**：以序號查手冊版本與適用序號段（N642SP = 172S8050 → 172S Information Manual Rev 5，172S8001 起）；不同序號段的手冊不得混用。該機與手冊構型的差異（如未裝輪罩）寫成 FDM 屬性，由試飛卡的 `handbook_configuration` 只在手冊點套用。
- **逐值出處、逐頁核對**：轉錄的表格每個數值附「圖號、頁、欄」，提交前逐頁對照原圖。
- **讀項目名稱，不只讀數字**：設備清單同一列的力臂可能屬於煞車總成而非輪軸；工程圖上的「ground line (static)」與槳弧高度也不是停放姿態或機高。
- **證據優先序**：該機型的原始照片與工程圖 > 研究推論 > 社群資料。衝突時照片勝出，並在設計檔記下被覆蓋的推論與照片出處。
- **實飛紀錄的可採性**：只有日期、風（RAP/HRRR 再分析沿航跡取樣）、轉速（可由錄影音軌的槳葉通過頻率讀出：二葉槳 rpm = 30 × Hz）與機號都確定時才進 `empirical_segments` 評分；否則移到 `excluded_empirical_segments` 並以 `excluded_because` 寫明原因與證據。

---

## 2. 命名與包結構 (Stage 2)

### 2.1 命名、子型號家族與外部塗裝／變體包匯入
- 目錄、套件 id、crate 名一律用**小寫機型代號＋子型號**：`vehicles/p38g`、`"id": "p38g"`、`sim-p38g`、`fdm/p38g.xml`。只有子型號造成動力、氣動或系統差異時才帶子型號（如 `vehicles/c172s` 為 180 BHP IO-360-L2A + 76×60 槳、2,550 lb MTOW；`vehicles/c172p` 為 160 BHP O-320-D2J + 75×57 槳、2,400 lb MTOW）。
- **機型家族（Variant Family）**：`sim_pack::VariantRegistry` 將同系子型號歸入同一 `family`（如 `family: "c172"` 下掛 `c172s` 與 `c172p`，並保留舊連結相容別名 `"c172"` → `"c172s"`）。每個子型號各自擁有 `Liveries/catalog.json`（`default_livery` 指派預設尾號：`c172s` 為 `"n642sp"`，`c172p` 為 `"d-ecjb"`）。
- **單一機身（序號、註冊號、部隊、機名）是塗裝或個別機 OpenUSD 覆蓋層，不是載具**：共用乾淨白模基礎舞台 `usd/c172s_airframe.usda` / `usd/c172p_airframe.usda`，每架機在 `usd/<tail>_livery.usda` 與 `usd/<tail>.usda` 以 `subLayers` 覆寫底色、投影條紋網格、註冊號字元與垂尾國籍章（由 `vehicles/tools/c172_livery.py` 從 `usd/<tail>_livery.json` 重生），不複製基礎網格、不寫進 crate 名稱。
- **外部塗裝與變體包動態匯入**：`sim_pack::LiverySpec`（宣告式 JSON：註冊號規範、底漆、條紋軌跡、垂尾色帶）與 `ExternalLiveryBundle` 支援在原生與 `wasm32` 執行期匯入外部塗裝（含可選 `.usda` 覆蓋層，或由 `sim_maplibre::usd_aircraft::compose_livery_stage` 即時射線投影到基礎機身網格），原生端亦支援透過 `VariantRegistry::import_variant_dir`、`SIMULATE_IMPORT_LIVERY` 與 `SIMULATE_IMPORT_VARIANT` 載入外部 `syspack.json` 變體目錄與塗裝。
- 改名時同步：目錄、crate 名、workspace member 與依賴別名、`syspack.json` 的 `id`/`name`、JSBSim `fdm_config name` 與 `dynamics.JsbsimXml`、塗裝目錄的 `aircraft_id`/`dlc_mount_point`、`sim-pack/src/builtin.rs` 的 `include_bytes!` 路徑、`simulate-app`/`sim-maplibre` 的符號。最後 `grep` 舊字串歸零並 `cargo check` 下游 crate。

### 2.2 依賴規則
載具 crate **不得依賴** `sim-maplibre`、`sim-render`、`wgpu` 或視窗系統；渲染適配層只消費載具 crate 產出的網格與動畫狀態。可依賴數學、序列化、`openusd`、`mbd-core`、`sim-core`、`sim-pack`、`sim-geo`、`sim-atmosphere`。提交前自行檢查 `Cargo.toml`。

### 2.3 目錄
```text
vehicles/<id>/
├── Cargo.toml / src/          # 幾何 crate（如有）
├── syspack.json               # 信任根：每個資產的 sha256 與大小
├── fdm/<id>.xml               # JSBSim 模型
├── design/*.json              # 數位化工程圖、螺旋槳規格（含 provenance）
├── geometry/landmarks.json    # 結構基準（§3）
├── performance/
│   ├── poh_benchmarks.json        # 試飛卡（§6）
│   ├── validation.json            # 通過與未通過的列（§4.5）
│   ├── airspeed_envelope.json     # V 速度（PFD/HUD 標記）
│   └── pitot_static_calibration.json  # 位置誤差（§5.2）
├── views/cameras.json         # 預設視角（§7.2）
└── Liveries/catalog.json + Models/Liveries/*.xml
vehicles/tools/
├── propeller_bem.py           # BEM 表格與調速器反演表
├── propeller_measured.py      # 實測表＋槳尖馬赫＋BEM 的拼接表
├── propeller_check.py         # 工具對 TR 640 的檢查、表格可重生檢查
└── syspack_rehash.py          # 修改資產後刷新 sha256/size 與儀表包 pin
```
- `syspack.json` 為 fail-closed：**任何資產修改後必須執行** `python3 vehicles/tools/syspack_rehash.py vehicles/<id>`。新增資產先在 `assets` 加條目再 rehash；builtin 載具同步在 `sim-pack/src/builtin.rs` 以 `include_bytes!` 掛上。
- **可重用零件**（發動機、螺旋槳）是獨立的靜態 syspack（`kind: Engine` / `Propeller`、`dynamics: None`，文件資產 `engine.json` / `propeller.json`）；載具以頂層 `parts` 的 `{role, id, version, manifest_sha256}` 釘住，同一 `role` 不得重複。同型發動機或螺旋槳被多機使用時抽成零件，不複製數值。
- `./scripts/check-structure.sh`：禁止 `mod.rs`/`utils.rs`/`helpers.rs`/`common.rs`；每檔 ≤ 500 行、`lib.rs` ≤ 100 行並以 `//!` 開頭、每函式 ≤ 80 行；非測試碼不得 `+= 1`。

---

## 3. 幾何 (Stage 3)

1. **數位化工程圖**：掃描不入庫。去掉細線（尺寸線、延伸線、中心線）後描出外形環，以英寸存入 `design/<drawing>.json`（範例 `vehicles/p38g/design/an01-75f-2_general_arrangement.json`）。每個視圖由延伸線**分軸**擬合 px/in（掃描非等向），以畫出的基準線（FRL）註冊，不用機首尖。每個尺寸標註存數值與像素出處；比線條還細的部位列為 `thin_feature_rings`，比對時不計。
2. **放樣**（`vehicles/p38g/src`）：`stations.rs` 的隔框常數表以 smoothstep 插值、超橢圓截面連接；主翼用真實 NACA 厚度與彎度線（翼根 23016 → 翼尖 4412），襟副翼留鉸鏈槽與鉸鏈軸；螺旋槳三態（靜止槳葉、高速槳盤、順槳）；動力艙、冷卻器與座艙蓋分片依批次區分。`cockpit_reservation.rs` 公開 `CockpitInteriorContract`（儀表板框、槽位、設計眼點、操縱件、座椅）。
3. **測試網格本身，不比自己的常數**：`mesh_silhouettes_match_the_an01_75f_2_general_arrangement` 按視圖算 IoU、最大與 95% 邊界偏差，限值設在現值之外作為棘輪，只許收緊；`built_mesh_measures_to_the_an01_75f_2_callouts` 量測網格上的槳平面、翼根、輪軸、尾翼等標註尺寸。
4. **FDM 用圖紙框架**：JSBSim 結構座標 x = FS、y = BL、z = 高於 FRL（in）。推力器在槳平面與推力線；BOGEY 在畫出的未受載輪胎底點，彈簧係數使靜壓縮等於圖上輪胎變形；停放高度由圖紙地面線與受載重心推導。`apps/simulate-app/src/vehicles/tests/gear_datum.rs` 比對輪底對接地點、槳盤對推力器、停放高度。
5. **結構基準**：`geometry/landmarks.json` 列出有來源的站位（槳盤、輪軸、翼根前緣…，每項附 `source`）與網格特徵規則（`feature.rule`）。載入時以最小平方擬合放置模型；殘差指出哪個特徵偏了，測試設殘差上限（`the_c172_is_placed_by_its_poh_landmarks_within_bounds`）。模型不跟隨重心。

---

## 4. 飛行動力學 (Stage 4)

### 4.1 質量
`emptywt` 用基本空重（含固定武器），其餘以 pointmass 分列：`pointmass[0]` 為**酬載槽**（飛行員），試飛工具調整它來達到試飛卡總重。燃油按真實油箱分列並指定 `<feed>`。manifest 的元件質量須與 JSBSim 重量一致。

### 4.2 渦輪增壓活塞發動機
JSBSim piston 以「每檔一個固定壓比 × 油門」建模增壓器，`ratedboost` 為調壓器上限：
1. `ratedboost = MAP_rated − 29.92 inHg`；起飛／戰鬥壓力用 `takeoffboost`（油門 > 0.98 才開放）。
2. **以多檔增壓代替連續廢氣門**（P-38G：3 檔，6,000 / 15,000 / 24,700 ft 自動切換），使低空油門可從怠速連續調到額定。
3. **最高檔額定高度**由臨界高度以上的進氣壓力衰減反推（P-38G：30,000 ft 得 36 inHg）；`ram-air-factor` 調低（0.2），因為臨界高度以上受渦輪轉速限制。
4. **增壓充氣加熱**：在 FCS 計算壓氣機出口溫度（η_c 0.65）與中冷器效率（0.5），以 `T_amb/T_charge` 縮放 `volumetric-efficiency`；同一溫度即化油器進氣溫度表讀數。少了這步，同進氣壓力下功率隨高度多出 30% 以上。
5. **自動混合比**：JSBSim 當量比為 `1.3 × mixture × p0/p_amb`；FCS 依手柄檔位寫入 `mixture-pos = φ/1.3 × p_amb/p0`。
6. 最後只用 `bsfc`（實為指示比油耗）縮放絕對功率，對準試驗的每發 BHP。

### 4.3 螺旋槳
- `vehicles/tools/propeller_bem.py` 從 `design/<prop>.json`（槳葉數、直徑、弦長與厚度分佈、設計進距比）生成 `C_THRUST`/`C_POWER(J, β_0.75R)`；`eta` 輸出用於檢查最高效率（WWII 金屬槳約 0.82–0.86）。
- **BEM 的適用範圍**：低進距比（J < 0.3，起飛滑跑）不可靠，靜態 Cp 只有 NACA TR 640 5868-9 實測的 40–75%（`propeller_check.py` 的 `STATIC_CP_RATIO`）。定距槳因此用 `propeller_measured.py`：J < 0.3 取 TR 640 實測表並加 TR 684 Fig. 18 的槳尖馬赫修正（實測在槳尖 M≈0.5，C172 靜態約 0.7），高 J 用 BEM（設計檔的 `bem_above`），中間線性過渡。`scripts/check-propellers.sh` 確保每張出貨表可由設計檔重生。
- 試驗報告的槳葉角若在其他半徑（42-12687：42 in 處 22.7–57.7°），依扭轉換算到 0.75R。高速槳尖馬赫損失用 `CT_MACH`/`CP_MACH`。
- **定速調速器放在 FCS**：內建的 `pitch -= ΔRPM·dt` 在配平迭代中振盪。螺旋槳保留 `minrpm`/`maxrpm`（內建調速器只在生成時的穩態求解起作用），FCS 以 `propeller_bem.py ... beta` 的反演表 β(J, Cp) 前饋求槳角（前饋功率取軸功率與「進氣壓力可提供功率」較大者），加慢速 RPM 積分修正，輸出到 `propulsion/engine[n]/blade-angle`。**必須公開 `propulsion/engine[n]/governor-rpm-req`**，試飛工具以此判斷定速槳。
- 順槳由 FCS 積分器驅動槳角到 87°；風車阻力與不對稱偏航由螺旋槳模型自然產生，不另寫阻力表。

### 4.4 氣動
- 失速 CLmax 依試驗失速 CAS（非 IAS）與重量反推；襟翼增量分開調。
- 冷卻風門阻力只計**超出試驗時中立位置**的增量。
- 槳後被吹的機身零件以螺旋槳誘導速度計滑流阻力（TR 640 p. 23；`propulsion/engine[n]/prop-induced-velocity_fps`），不用自由流動壓。
- 無俯衝恢復襟翼機型需 Mach tuck 與尾翼效能衰減；無助力副翼依動壓衰減滾轉權限。

### 4.5 校準原則
1. **先加有來源的物理，再擬合**，順序：實測螺旋槳表＋槳尖馬赫 → 滑流阻力 → 發動機兩點校正（廠商全油門曲線的兩個轉速，如 Lycoming 2,300 與 2,700 rpm）→ 手冊技術（起飛「機尾略低」、在 VLOF 離地；全富油／最佳功率／EGT 貧油程序由試飛工具照手冊飛）→ 最後才擬合少數參數（CD0、誘導阻力因子、弦長比例），並聯合重擬受同一修正影響的列。
2. **每個值標為 sourced / fitted / assumed**，寫在設計檔的 `provenance`（範例 `vehicles/c172s/design/mccauley_1a170e_jha7660.json` 與 `vehicles/c172p/design/mccauley_1c160_dtm7557.json`，另有 `stand_in` 與 `not_met`）。被擬合到某列的參數不得再把該列當獨立證據。
3. **不放寬容差、不加無來源的修正係數**。無法同時滿足的列寫入 `performance/validation.json`（`passing`、`failing`、`failing_checks`、`known_table_steps`）；測試逐列比對紀錄，列的結果改變而紀錄未更新即失敗。
4. 排查偏差時逐一以量測排除假說（離地速度、摩擦係數、手冊裕度、工具本身），再動模型。

---

## 5. 大氣資料、儀表包與座艙 (Stage 5)

### 5.1 分層
| 層 | 位置 | 職責 |
|---|---|---|
| 物理與機構模型 | mbd 倉庫的 `mbd-airdata`（`no_std`） | ISA、可壓縮皮托關係、管路延遲、靜壓位置誤差、各儀表機構與指針二階動態 |
| 契約與信任 | mbd 倉庫的 `syspack` | `PitotStatic` 裝置、`instrument_pack` 引用、`kind: InstrumentPack` 與 `instruments.json` 驗證、零件 pin |
| 載具 | `vehicles/<id>` | 皮托靜壓裝置位置、PEC、引用哪個儀表包、座艙面板座標框 |
| 共用儀表包 | `instruments/<pack-id>/` | 儀表清單（真實型號、量程、非線性刻度、色弧、機構常數、儀表誤差）、預設面板佈局；多機共用 |
| 顯示 | Indicate 倉庫／3D 機械儀表幾何 | 只消費狀態（指針角、讀數），不計算物理 |
| 設計 | ace-rs 倉庫 | 新設計時估算 PEC、選擇儀表包並輸出上述資產 |

### 5.2 位置誤差 (PEC)
`performance/pitot_static_calibration.json` 每條襟翼曲線為 `{kias, kcas}` 點，`kias` 為**水柱空速**（只含位置誤差）。試驗範圍外的點在 `reference_document` 中標示為假設。

### 5.3 儀表包
儀表包本身是 syspack：`kind: "InstrumentPack"`、`dynamics: "None"`、`devices: []`、一個 `panel` 元件，以及資產 `instruments.json`（schema 在 `syspack::instrument_pack`，`deny_unknown_fields`）：
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
- `kind`：`Airspeed`、`Altimeter`、`VerticalSpeed`、`PressureGauge`、`Tachometer`；刻度為單調的非線性 `scale`，色弧／色線用 `markings`。
- `source` 綁定語法封閉，打錯即載入失敗：`pitot_static.<裝置 id>`、`engine[n].<量>`、`system.<路徑>`。
- `name` 只寫能查證的型號；查不到時寫功能與量程，不編造型號。範例：`ga-analog-six-pack`（C172）、`usaaf-1943-fighter`（P-38G）。

### 5.4 載具端綁定
- `syspack.json` 的 `devices` 加 `PitotStatic`（`pitot_body_m`、`static_body_m` 為前-右-下座標，`probe`、`total_line_tau_s`/`static_line_tau_s`）。
- 頂層 `instrument_pack: { "id", "version", "manifest_sha256" }` 釘住儀表包；`LoadedSyspack::load` 驗證 pin，並檢查每個 `pitot_static.<id>` 對應到本機裝置。
- 修改後依序 rehash：先 `instruments/<pack>`，再引用它的載具（`syspack_rehash.py` 會刷新 pin）。
- 驗收：`crates/sim-pack/src/instruments/tests.rs` 在 127 mph CAS 重現 42-12690 報告的 115 mph 指示。新機型加入同類「試驗報告一列 → 指示讀數」測試。

### 5.5 座艙與儀表原則
- **不重複濾波**：機構延遲與儀表誤差只在 mbd 機構模型；值→角度只在顯示層；指針只接角度。沒有訊號就顯示 OFF，不假設 0。
- **錶面與字體**屬於儀表包：刻度、標記、指針與錶面字樣在包內宣告，字體以雜湊釘住的資產引用，不寫死在載具或渲染器。
- **面板佈局**由現存實物面板或照片量測孔位（§7.3 反投影），來源只引用 URL；查不到的項目列為未知，不補想像的儀表。
- **操縱件**以關節（軸、每單位行程）由 FDM 的**指令**而非舵面位置驅動；無來源的行程寫成具名的假設常數並註明。

---

## 6. 閉環試飛 (Stage 6)

### 6.1 試飛卡 `performance/poh_benchmarks.json`
- `stalls`：`target_kcas`、`target_kias`（水柱 IAS，與 PEC 一致）、重量、襟翼、坡度。
- `climb_points`：高度、重量、`climb_kcas`、`target_fpm`；增壓機加 `target_map_inhg`。
- `cruise_points`：高度、重量、`target_rpm`（曲軸轉速）、`target_ktas`；增壓機加 `target_map_inhg`。
- `takeoff_points`（手冊短場距離）、`static_runup`、`empirical_segments`（§1.5）、`handbook_configuration`。
- `rated_bhp` 為全機總額定功率；`reference_document` 寫明報告編號、版次與適用序號。

### 6.2 試飛工具的行為（`crates/sim-pack/src/calibration`）
- 每個有動力的試驗點**使用新載入的模型**，不帶入前一點的發動機、調速器或燃油狀態。
- 油門、混合比、螺旋槳手柄寫入**每一台發動機**（`fcs/*-cmd-norm[n]`）；不帶索引的名稱只作用於 0 號發動機。
- 定速槳以螺旋槳手柄保持 `target_rpm`、以油門保持 `target_map_inhg`；定距槳以油門保持 rpm。
- 爬升：60 s 飛行、最後 15 s 量測**能量高度** `h + V²/2g` 的變化率。
- 平飛：以 6 s 為一段持續飛行，直到相鄰兩段平均真空速相差 < 0.05 kt（至少 30 s、最多 600 s），取最後一段；固定飛行時間會讓結果取決於起始速度（`tests/level_speed.rs`）。
- 起飛（`takeoff.rs`）：煞車全功率、鬆煞車、依手冊短場技術抬輪，量滑跑與 50 ft 距離，離地速度有守護測試。

### 6.3 指令
```sh
# 全卡（Markdown 報表）
cargo run -p ironbird -- --mode calibrate-dir --package-dir vehicles/<id>
# 全卡逐列列印（改 FDM 後逐行比對）
cargo test --release -p sim-pack --test card_report -- --ignored --nocapture
# 回歸測試
cargo test --release -p sim-pack --test p38g_flight_test
cargo test --release -p sim-pack --test n642sp_calibration_workflow
# 發動機工作點探針（調參用）
cargo test -p sim-pack --test p38g_flight_test engine_operating_points -- --ignored --nocapture
```
調參順序：先用探針對準每發功率與進氣壓力（§4.2），再看平飛速度（阻力、螺旋槳效率），最後看爬升、起飛與失速。每次修改 FDM 後先 rehash。

---

## 7. 視覺 (Stage 7)

### 7.1 外部表面原則
- 塗裝與標記以**圖紙英寸或網格量測的錨點**（機首、翼前後緣、尾桁站位）定位，不用模型絕對座標；側面標記從兩側測試鏡像讀向。貼花沿蒙皮法線偏移 0.6–0.8 mm，弧面圖案做重心座標細分。
- 顏色寫**規範名稱**（如 OD 41、Neutral Gray 43），RGB 標為近似；查不到的標記不畫，列為待查。
- 面板線由可重跑的工具自有授權的三視圖描出，不手改，並以工程圖站位測試。磨損由位置、法線、遮蔽計算，以同單位的黑白歷史照片校準對比與輪廓，不判斷色相。
- 照片與貼圖影像不入庫；程序化生成的表面在建置時產生。

### 7.2 預設視角 `views/cameras.json`
- 列入 `syspack.json` 資產，由 `sim_pack::CameraViewSet` 解析；無檔案時依 JSBSim 翼展自動生成。每個視角有 `id`、`title`、`fov_y_deg`、`airframe`（`external` / `pilot_eye` / `cabin_window` / `hidden`）與 `placement`：`chase {back_m, up_m}`、`look_at {eye_frd_m, target_frd_m, up_frd}`、`cockpit_look {yaw_deg, pitch_deg, eye_shift_frd_m}`、`spotter {grid_step_m, lateral_m, height_m, lead_time_s, frame_span_m}`。
- 位置為前-右-下、公尺、相對於畫出的模型原點。座艙視角以座艙契約的設計眼點為原點加位移，**不要把眼點複製進 JSON**。
- 戰鬥機基本組：追尾、僚機 3/4 後、低空前側掠過、側視、俯視、地面觀察、翼尖向內；座艙前視、瞄準具、各發動機、回望、儀表板特寫。客艙機加窗外視角。
- 近距離 `spotter` 的 `lead_time_s` 設 0，否則飛機被帶出畫面。

### 7.3 照片對齊（外部與座艙同一方法）
程式在 `apps/simulate-app/src/cockpit_rig/verify_shots/photomatch`，照片資料在其中的 `photos.json`（URL、授權、尺寸、起落架狀態、太陽估計、像素地標）。
1. **先解相機，把模型對齊照片**：只用不會被修改、且不共面的固定結構當地標（翼尖、垂尾頂、槳轂尖、輪底；座艙內用座艙框接點、風擋柱、儀表板外框角），至少 6 個、最好 9–11 個且深度分散。針孔模型、主點固定在影像中心（放開主點會發散），Levenberg–Marquardt 由多個起始視點求解；座艙照片由設計眼點附近的頭部範圍起始（`cockpit_starts`）。RMS ≤ 影像對角線 1.6% 為佳，> 4% 測試失敗。
2. **辨識**照片中每個儀表／部件（可讀的錶面、大小、刻度佈局、研究資料），附 id 與信心。
3. **反投影放回模型**（`backproject.rs`）：自解出的相機經特徵像素射線與平面求交（儀表板平面、對稱面）、與網格求交（貼附於表面的標記與零件），兩張以上照片則三角定位（離開表面的零件）。更新佈局並引用照片像素，保留逐項重投影殘差測試；地標隱含的 3D 偏移 > 0.15 m 視為模型真實差異。
4. **機械件看輪廓**：駕駛盤、操縱桿、座椅、開關盒、起落架等在對齊視角疊輪廓，逐件修到吻合並記錄尺寸來源。
5. **色調只在對齊視角比較**：以解出的相機與估計的太陽渲染，與照片同區塊比灰階；光照只用晴天照片判斷。
6. 照片只引用 URL（公有領域或 CC BY），影像放在庫外。地面照片以空中、放輪、順槳姿態渲染（解出的相機可能落在停放模型的地形內），並等起落架行程結束。

### 7.4 指令（`--release`，各 worktree 自己的 `CARGO_TARGET_DIR`）
```sh
# 輪廓與尺寸標註（差異圖：紅＝只有圖紙，藍＝只有網格）
SIMULATE_VERIFY_OUT=<dir> cargo test --release -p sim-p38g
# 與原圖對齊的正射疊圖、稽核截圖、情境截圖、每個宣告視角
SIMULATE_VERIFY_OUT=<dir> cargo test --release -p simulate-app --bin simulate-app render_fidelity_audit_ortho_overlays -- --ignored
SIMULATE_VERIFY_OUT=<dir> cargo test --release -p simulate-app --bin simulate-app render_fidelity_audit_shots -- --ignored
SIMULATE_VERIFY_OUT=<dir> cargo test --release -p simulate-app --bin simulate-app render_p38_third_person_verification_shots -- --ignored
SIMULATE_VERIFY_OUT=<dir> cargo test --release -p simulate-app --bin simulate-app render_vehicle_view_sets -- --ignored
# 照片對齊：解算測試；渲染並列圖、疊圖、差異圖與 solutions.json
cargo test --release -p simulate-app photomatch
SIMULATE_PHOTOMATCH_DIR=<images> SIMULATE_VERIFY_OUT=<dir> cargo test --release -p simulate-app render_photo_matches -- --ignored
```
截圖逐張目視；修改一機的表面時，其他機的參考截圖須像素不變（`magick compare -metric AE`）。

---

## 8. 提交前門檻與已知陷阱

```sh
cargo fmt --check        # 或 cargo fmt -p <crate>；不要 --all
cargo clippy -p <vehicle-crate> -p sim-pack -p sim-maplibre -p simulate-app --all-targets -- -D warnings
cargo test --release -p <vehicle-crate> -p sim-pack
for check in scripts/check-*.sh; do "$check"; done
```
完整 CI 為 `./scripts/ci.sh`（另含 wasm32 clippy、文件門檻與 release 建置）。

陷阱：
- **`cargo fmt --all`** 會連同以路徑依賴的兄弟 repo 一起格式化，改到別人的工作區。
- **每個 worktree 用自己的 `CARGO_TARGET_DIR`**：共用 target 時同名 crate 互相覆寫且仍顯示 Fresh，測試與截圖會跑到別的 worktree 的程式；建置前 `df -h ~`，完成後在自己的 target `cargo clean`。
- **分支只在自己的 worktree 內以 `git rebase` 更新**；不要在主 checkout 執行會重設其他分支的指令。兄弟 repo 漂移時 rebase 到帶有新 pin 的分支，不要用快照或路徑覆寫建置。
- **jsbsim-rs 活塞發動機在同一模型第二次 `run_ic` 後不會重新運轉**（`propulsion/set-running` 不生效）；試飛工具因此每點重新載入模型。
- **生成時的槳距**：`EngineStartMode::Running` 跑 JSBSim 的發動機穩態求解，不執行 FCS；FCS 調速器的機型必須保留內建調速器，否則停機坪上以細距起步並自行滑走。
- **JSBSim 配平**（`do_trim(Full)`）在 FCS 驅動槳角的機型上不收斂，且增壓調壓器的平坦區使油門軸導數為零。配平失敗不破壞狀態；需要穩定飛行的測試改用短時間閉環穩定。
- **JSBSim 把全富油（φ 1.3）當最佳功率混合比**，且燃油流量 = 指示功率 × `bsfc`；全富油爬升與油耗須照手冊程序飛並對照，不能直接讀。
- JSBSim 正方向舵使機首向左、正前輪轉向使機首向右；踏板到前輪的連桿增益為 −1。
- 這台機器的 `cp` 是互動式別名；腳本中覆寫檔案用 `\cp -f`。
