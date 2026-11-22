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

本文件是新增或精修 `Simulate` 載具的標準流水線，以 [`vehicles/p38g`](p38g/Cargo.toml)（Lockheed P-38G-13-LO）為完整範例，[`vehicles/c172s`](c172s/syspack.json)（N642SP，172S8050）與 [`vehicles/c172p`](c172p/syspack.json)（D-ECJB，Reims F172P）為單發輕型機子型號與尾號塗裝分離範例，[`vehicles/apollo-11`](apollo-11/Cargo.toml) 為非飛機範例。依任務範圍選擇階段，檢查其實際依賴；**純模型精修可在 1、2、3、7 間迭代，不以完成 FDM、座艙或試飛為前提**。外形、機械動畫準備與飛行性能各自驗收，不以其中一項通過代替另一項。

| 階段 | 產出物 | 驗收門檻 |
|---|---|---|
| 1 原始資料考證 | 數值附來源位置、單位、基準與構型；性能數據另附工作條件及限制 | 無來源的數字不得進入 FDM 或試飛卡；轉錄表逐頁核對 |
| 2 命名與包結構 | `vehicles/<id>/` 目錄、crate、`syspack.json` | `cargo check`、`syspack_rehash.py`、builtin 載入測試 |
| 3 幾何 | 數位化圖紙、網格、`geometry/landmarks.json` | 匯出網格的輪廓、剖面、尺寸與結構基準；涉及 FDM 時再核對接地點 |
| 4 飛行動力學 | `fdm/<id>.xml`、設計輸入 `design/*.json` | 引擎探針、螺旋槳表可由設計檔重生 |
| 5 大氣資料、儀表與座艙 | 皮托靜壓裝置、PEC、儀表包引用、座艙契約 | PEC 載入與指示空速測試 |
| 6 閉環試飛 | `performance/poh_benchmarks.json`、`performance/validation.json` | 試飛卡逐列結果與驗證紀錄一致 |
| 7 視覺 | 表面、`views/cameras.json`、照片對齊 | 對齊視角下逐張目視、重投影殘差測試 |

## 接手與逐步交付

每個可交接步驟都要能由另一個代理接續：讀取本文件、相關工具與任務範圍，完成修改後執行相應驗證並讀取結果，再交付可重現的產出。步驟結束時核對本文件與實際流程；有可重用的新知或失效指令就原位更新，沒有流程變化則保留原文。

- **確認工作對象**：辨識構型／塗裝、原始生成器、正式安裝包與候選版本。讀取現有變更及授權邊界；模型檢查、模擬器程式修改、啟動場景和外部發佈分別依任務授權，不能由本文件推定額外權限。
- **交接可執行的依據**：在任務對話、PR 或既有驗證輸出交代實際檔案路徑、必要的工具版本／依賴、工作目錄、重建與驗證命令、來源連結及頁／原圖像素座標、單位／基準／不確定性。每種交付列明適用閘門及未覆蓋項；驗證結果綁定輸入、生成器、檢查器與產出的雜湊及對應截圖，包含讀過的命令結果、退出碼、通過範圍與尚未解決的失敗。單靠「已驗證」或一張漂亮截圖不足以交接。
- **候選與正式交付分開**：未通過整體驗收的候選不得覆寫正式資產。接手時重新核對雜湊與適用範圍；獨立零件、整體裝配、各 LOD、材質、目的模擬器的動作各自給出結論。截圖、GLB、USD、生成器與測試所用二進位必須對應同一候選，不能以快取或另一個 worktree 的成功代替。
- **留下可重建的工作**：驗收後把所需生成邏輯和有效檢查納入既有工具及資產打包流程，確認交付的來源能重建結果；打包清單須包含間接匯入的模組及必要資產，暫存目錄不可成為正式交付的唯一依賴。本文件保存穩定方法與使用方式，任務進度、日期、臨時路徑和當次測試數不追加成文件日誌。交接要明示未完成項及下一個可執行檢查，不把未驗證範圍改稱完成。

---

## 1. 原始資料考證 (Stage 1)

先鎖定**具體批次／子型號或序號**（P-38G-13-LO 而非泛稱 P-38；G 型無俯衝恢復襟翼、無液壓副翼助力、機翼前緣中冷器），不同批次的動力、氣動與系統不得混用。

### 1.1 文獻優先順序
1. **官方飛行試驗報告**（含實測條件，權重最高）：如 AAF Materiel Center Memorandum Report FS-M-19-1538-A（P-38G-1 42-12687：各高度平飛速度、爬升率、進氣壓力、每發 BHP 與限制因素）、FS-M-19-1492-A（42-12690：失速速度、皮托靜壓頭位置與位置誤差表）。掃描 PDF 需逐頁讀圖。
2. **飛行操作手冊 / T.O. / POH**（限制速度、程序、性能表）。
3. **製造商規格書**（保證性能、設計重量）與**工程圖**（總體佈置、站位）。區分有圖號、適用構型及尺寸標註的製造／維修圖，與供模型製作用的三視圖、作者重繪及套件說明書；後者可作外形參考，不能宣稱製造公差精度。
4. **NACA 報告**（翼型、螺旋槳實測、可壓縮性）。
5. **遊戲或模擬器社群資料**：未附原始出處的參數只能作交叉提示，不得直接作 FDM 數值來源。依原始文件的作者、用途與可核驗性分類，不依託管網站分類；社群網站收錄的作者圖紙仍可作帶不確定性的模型外形參考。

### 1.2 數據的適用條件
幾何量測記錄單位、基準、構型／單機、來源位置與不確定性。性能數據另需重量、重心、構型（襟翼／起落架／冷卻風門）、高度、功率設定（進氣壓力 inHg + rpm，而非「全油門」）、混合比及**限制因素**；純外形量測不要求無關的發動機工作點。例：42-12687 在 22,600 ft 以 44.7 inHg 達 404 mph，該高度即渦輪臨界高度；24,400 ft 起受化油器進氣溫度限制；30,000 ft 起受渦輪轉速限制。限制因素直接決定增壓模型（§4.2）。

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
- **證據衝突**：先核對機號、日期、改裝、受載姿態、圖紙適用性與掃描／鏡頭畸變。尺寸依可靠標註，外觀用多角度同構型照片交叉檢查；不能一律讓照片或圖紙勝出。尚不能解開的衝突保留來源與不確定性，不以美觀推論替代實物證據。
- **生成圖與其他模型**：AI 圖像、重建模型與套件示意可提示待查特徵，不能作為獨立尺寸、剖面或機構證據。外部流程先核對程式、授權、依賴、服務成本及真正輸出；可採用可重現的建模方法，但其展示、數字或成功訊息仍須由適用的原始資料與本專案檢查獨立驗證。
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

1. **數位化圖紙**：掃描不入庫。去掉尺寸線、延伸線、中心線後描出外形環，以明確單位存入 `design/<drawing>.json`（範例 `vehicles/p38g/design/an01-75f-2_general_arrangement.json` 使用英寸）。由尺寸標註的延伸線校正比例與已證實的掃描非等向變形，以畫出的基準線（FRL）註冊，不用機首尖；不能為讓模型吻合而任意拉伸圖紙。每個標註存數值與像素出處；比線條還細的部位標記解析度限制，不計入無法可靠量測的指標。掃描像素階梯不等於製造折角；可在已記錄的不確定帶內擬合平順曲線，但須對實際匯出邊界回量原圖，並重測相鄰蒙皮與內壁的交接。每個視圖明示觀察方向及畫面右／上軸，將實際匯出輪廓經同一相機投影後比對；前視圖的畫面橫軸不可直接當作機體橫軸。左右鏡像與不同站位各自重測，不能用一個發動機位置的描線驗收其他位置。
2. **剖面與放樣**：追蹤各站位的寬、高、最寬處高度、肩部、下頦與曲率轉折，不能只以圓／橢圓加長寬比代替非橢圓剖面。插值應保留有來源的站位，區分描圖／量測剖面與僅供投影或估計的導引站；不能把後者當作額外實測約束。用斜視、頂視及中性反射材質檢查站間凹陷、鼓包與曲率轉折，不能只在剖面站驗收。風擋逐片核對角點、傾角、寬高、窗框交點及實物的平板／曲面形態；平板玻璃另核對邊界直線、窗框內外邊與蒙皮貼合；以正、側、頂與斜視共同確認玻璃數量和連通關係，不能只憑單一投影補窗或窗柱。開口後不得仍留不透明蒙皮擋住玻璃，浮起的框條不能代替真正的接合面。機翼的平面形、翼型、厚度、扭轉、上反角與翼根整流分別取證；P-38G 的 NACA 23016 → 4412 等資料僅適用該構型，不外推給其他機型。其 `cockpit_reservation.rs` 公開 `CockpitInteriorContract`（儀表板框、槽位、設計眼點、操縱件、座椅），外形精修須保留座艙契約。
3. **測試實際匯出物**：從組合塗裝後重新載入的 USD／GLB 及各 LOD 切出剖面、正投影輪廓，量測尺寸、槳平面、翼根、輪軸與窗角；同時驗證封閉性、幾何面朝向、自交、材質邊界與表面縫隙，不只比生成常數；共享邊須沿同一組頂點分段，平滑法線不能掩蓋折疊或重疊面。另以中性材質和反射光照分別檢查曲面與法線連續性；非等距放樣須檢查對稱點的法線，避免三角形面積與對角線偏置造成假凹凸。匯出後對完成塗裝合成的來源重測法線，不以基礎網格代替被塗裝覆寫的表面；傳遞法線時須保留其他頂點屬性與硬邊。確認烘焙圖集沒有在窄窗框等細片留下未覆蓋的暗斑。以同一描圖點建模再回量，只證明構造與匯出一致性；獨立外形驗證須使用未參與擬合的特徵、視圖或來源。報告 IoU、最大／95% 邊界偏差及逐項殘差；門檻依來源解析度與使用距離決定，不能只在當前誤差外設一條線就宣稱準確。範例測試為 `mesh_silhouettes_match_the_an01_75f_2_general_arrangement`、`built_mesh_measures_to_the_an01_75f_2_callouts`。
4. **涉及 FDM 時的基準**：JSBSim 結構座標 x = FS、y = BL、z = 高於 FRL（in）。推力器在槳平面與推力線；BOGEY 在畫出的未受載輪胎底點，彈簧係數使靜壓縮等於圖上輪胎變形；停放高度由圖紙地面線與受載重心推導。`apps/simulate-app/src/vehicles/tests/gear_datum.rs` 比對輪底對接地點、槳盤對推力器、停放高度。
5. **結構基準**：`geometry/landmarks.json` 列出有來源的站位（槳盤、輪軸、翼根前緣…，每項附 `source`）與網格特徵規則（`feature.rule`）。載入時以最小平方擬合放置模型；殘差指出哪個特徵偏了，測試設殘差上限（`the_c172_is_placed_by_its_poh_landmarks_within_bounds`）。模型不跟隨重心。
6. **細節與可動件**：維修口、鉸鏈、鎖扣、通風口、輪艙、門板、支柱、扭力臂、煞車與管路依照片及維修圖定位；不可用均勻面板格、鉚釘或想像管線填滿未知區域。起落架區分受載／伸展行程，檢查輪軸、輪胎截面、門板與收放掃掠空間。螺旋槳核對葉數、旋向、前後槳距、弦長、厚度、扭轉與除冰區；每片槳葉的變距軸、旋轉中心及基準槳角獨立於整個轉子的旋轉軸，對轉槳的停放相位不是固定機械約束。保留可動件的獨立網格與必要樞軸；舵面須從翼面切出而非覆在完整翼面上，輪艙與艙門須有實際開口。抽樣檢查中立、過渡與端點姿態的蒙皮、門緣、支柱、輪胎干涉，未知行程與收放次序明示未驗證。重新載入後核對渲染器是否實際採用樞軸、旋向與零位，防止名稱匹配誤觸其他機型的動畫；匯出 metadata 或預覽轉動不等於運行時正確。這是模型準備，不代表調速器、順槳、液壓系統或 FDM 已驗證。
7. **尺寸預算**：依預期視距、同屏數量、動畫及目標平台設定三角形、材質、貼圖與檔案大小預算。優先保留剪影、剖面、窗框和機械軸線；LOD 可刪除不能分辨的細節，不能把輪轂壓成多角塊、讓塗裝裂開或移動接地面。每次精簡後對實際匯出物重測，精細版與遊戲版分別驗收。

### 3.1 基準變更與機構驗收

- **局部視圖配準**：視圖缺少可共用的標示基準時，可用與待驗輪廓分離的同名實體特徵作局部配準；只解有足夠約束的自由度，記錄原圖座標、殘差與不確定性，不以模型或待驗輪廓殘差反調，也不外推為全機基準。配準後仍不一致的輪廓須保留其來源衝突。
- **先辨識物理特徵**：圖紙註冊錨點、板邊、孔中心、轉子平面與固定殼體端面各自命名。修正某軸心不代表整個宿主外形都應平移；沿依賴關係重新檢查相鄰罩殼、整流圈、進氣口、開口與樞軸。描的是蒙皮、加強板、孔緣還是後方零件須由其他角度確認，不能把重疊線條當成剖面凹陷。共用縫線的墨跡只定位接縫，不能推出零運動間隙；估計的間隙與壁厚須獨立標示。
- **檢查實際接合面**：銷軸要穿過真正的孔，並核對徑向間隙、軸向咬合與端部保持；畫一個圓盤、墊圈或遮住穿透不算接合。布林運算後重查封閉性、面朝向、薄長三角形、法線與材質邊界；保存不應改動的外形站位，不能靠平滑著色藏起相交或裂縫。
- **以機構約束驅動**：連桿兩端都附著於其接頭，剛性連桿長度保持不變；作動筒的兩端、桿／筒分離、伸縮量與限位同時成立。起落架收放、緩衝壓縮、輪胎轉動、煞車反力架及艙門各有對應父子關係；槳葉變距軸獨立於轉子自轉軸。USD／Blender 的 Z-up 與 GLB 的 Y-up 轉換後重測實際軸向與角度，任意 metadata 向量不會自動跟隨網格換基底。
- **掃掠整個裝配**：先測停放／零位，再測過渡、端點及可能同時發生的動作，包括可動件對固定蒙皮、艙口、進氣唇與其他組件。不能因共用骨架或初始姿態已相交就自動豁免；允許接觸的對象須有具體機構依據。列明實際納入的零件、篩選規則與動作範圍，不能把具名局部檢查描述成全機碰撞驗證。固定取樣須說明覆蓋的狀態；可計算時用連續掃掠的保守界。正的間隙下界可證明該模型在相應範圍內不相交；負的保守界只表示尚未證明安全，須定位實際干涉，不能直接當成穿透深度。
- **失敗必須阻止驗收**：無效作動筒／連桿解、超行程、脫離接頭或不允許的相交應使檢查失敗，不能只印出結果、截斷數值或用「解無效」繞過斷言。適用時以已知失敗的姿態確認檢查確實能攔截缺陷。零件通過後仍要重測完整匯出裝配與各 LOD，不能以局部成功掩蓋外圍固定件的干涉。執行 Blender 腳本時指定 `--python-exit-code 1`，讀取實際輸出；預設退出碼不足以判定 Python 腳本是否成功。
- **目的程式驗證**：需要模擬器動作時，從其實際載入與指令路徑驅動同一匯出包，量測頂點、鉸軸、銷孔、輪軸和門緣是否按約束運動。資產格式驗證、Blender 預覽、骨架或 metadata 存在各有用途，但均不證明目的程式已執行該動作；純模型任務可交付明確標示的模型驗證範圍。

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
- 塗裝與標記以**圖紙站位或網格量測的錨點**（機首、翼前後緣、尾桁站位）定位。優先在實際蒙皮上分割材質或烘焙紋理，圖案跨剖面與接縫仍須貼面。多視圖遮罩求交前，先區分實際材質分界與宿主剪影；剪影用於約束幾何，不能把描線誤差裁成連續塗裝中的假缺口。以網格分割材質時，邊界須與實際三角形求交，不能以重心判斷後整片著色；驗收同時檢查目標區域覆蓋與相補表面的保留。數值容差施於預定邊界，不以細碎殘差的厚度代替位置誤差；若引擎需要貼花偏移，依深度精度採最小必要值，近景檢查漂浮、穿透、假陰影與閃爍。雷達罩的外形屬於結構；其顏色分界不得靠疊一個近似形體實現。
- **字樣隨單機與年代核對**：記錄大小寫、筆畫／襯線、數字變體、字距、弧面扭曲及左右側差異，不能全機共用一個「軍用字體」就算符合。先校正照片透視，再比字形輪廓；現成字體無法吻合時可從清晰照片描矢量輪廓，保留出處與不確定區。以可換字體／圖形及側別的塗裝資料描述，確認授權可隨資產發佈；正反側都要測試讀向，不鏡像文字。
- 顏色寫**規範名稱**（如 OD 41、Neutral Gray 43），RGB 標為近似；查不到的標記不畫，列為待查。
- 面板線與維修口從適用圖紙和多角度實物照片描出，以可重跑工具生成並對站位核驗；不同面板的縫寬、鉸鏈與鎖扣有區別，不用通用網格代替。金屬、塗漆、介電罩、玻璃、橡膠和熱影響區分開建材質，以照片約束粗糙度、透光、刷紋與磨損；黑白照片只判斷對比與輪廓，不判斷色相。烘焙後重新載入成品，確認共用材質及未併入貼圖的可動件沒有殘留烘焙用的自發光材質。
- 參考照片原檔放在庫外；最終模型所需的貼圖僅在具備重發權時列入包資產，保留授權與出處。程序化表面在建置時產生。

### 7.2 預設視角 `views/cameras.json`
- 列入 `syspack.json` 資產，由 `sim_pack::CameraViewSet` 解析；無檔案時依 JSBSim 翼展自動生成。每個視角有 `id`、`title`、`fov_y_deg`、`airframe`（`external` / `pilot_eye` / `cabin_window` / `hidden`）與 `placement`：`chase {back_m, up_m}`、`look_at {eye_frd_m, target_frd_m, up_frd}`、`cockpit_look {yaw_deg, pitch_deg, eye_shift_frd_m}`、`spotter {grid_step_m, lateral_m, height_m, lead_time_s, frame_span_m}`。
- 位置為前-右-下、公尺、相對於畫出的模型原點。座艙視角以座艙契約的設計眼點為原點加位移，**不要把眼點複製進 JSON**。
- 戰鬥機基本組：追尾、僚機 3/4 後、低空前側掠過、側視、俯視、地面觀察、翼尖向內；座艙前視、瞄準具、各發動機、回望、儀表板特寫。客艙機加窗外視角。
- 近距離 `spotter` 的 `lead_time_s` 設 0，否則飛機被帶出畫面。

### 7.3 照片對齊（外部與座艙同一方法）
程式在 `apps/simulate-app/src/cockpit_rig/verify_shots/photomatch`，照片資料在其中的 `photos.json`（URL、授權、尺寸、起落架狀態、太陽估計、像素地標）。
1. **先解相機，把模型對齊照片**：使用已獨立核實、深度分散且不共面的結構地標（翼尖、垂尾頂、槳轂尖、輪軸；座艙用框接點、儀表板外框角），至少 6 個、最好 9–11 個。被修改的窗角、鼻部剖面等作為保留驗證點，不同時拿它們擬合相機再宣稱吻合。針孔模型由多個起始視點求解；有鏡頭／裁切資料時約束焦距、主點及畸變，沒有證據時不任意放開這些參數。先比對原檔尺寸、EXIF 與裁切／拼接資訊；經重採樣或拼接但投影未知的照片，不可直接用 EXIF 焦距推導像素焦長，也不能把單一針孔相機的擬合殘差全歸咎於模型。檢查左右手座標、相機高度、拍攝方向與視場。座艙照片可由設計眼點附近的頭部範圍起始（`cockpit_starts`）。記錄擬合點及保留點的逐點誤差；既有 1.6%／4% 對角線 RMS 門檻只能篩除明顯失配，不能證明風擋或剖面已達近景精度。
2. **辨識**照片中每個儀表／部件（可讀的錶面、大小、刻度佈局、研究資料），附 id 與信心。
3. **反投影放回模型**（`backproject.rs`）：自解出的相機經特徵像素射線與平面求交（儀表板平面、對稱面）、與網格求交（貼附於表面的標記與零件），兩張以上不同方向照片則三角定位（離開表面的零件）。用正面、側面、斜視互相約束，不能為吻合單張照片移動整個機翼或引擎。更新佈局後重新檢查保留點與剖面；3D 偏移先和相機／像素取點不確定性比較，再判斷是否是形狀錯誤，不把固定 0.15 m 套到每種零件。
4. **機械件看輪廓**：駕駛盤、操縱桿、座椅、開關盒、起落架等在對齊視角疊輪廓，逐件修到吻合並記錄尺寸來源。
5. **色調只在對齊視角比較**：以解出的相機及對應的直射／漫射照明渲染，與照片同區塊比較；曝光、白平衡、天空反射與鏡頭差異不能用來掩蓋幾何誤差。先以中性材質核對外形，再驗證材質。遇到疑似孔洞或相交的暗斑，先從實際匯出物以同視角的射線命中及材質／物件識別定位表面，再用中性材質與受控照明區分幾何缺口、材質分區及陰影／反射。
6. 照片引用來源與授權，參考影像放在庫外；只有具備重發權的圖案或紋理才可隨資產交付。渲染應匹配照片的受載姿態、輪門位置、槳葉變距與相位，不一律強制空中／順槳。相機落入地形先核對地面基準、單位與解算結果；純形狀比對可隔離地形，但不能把這種渲染當成實際停機坪接地驗證。

### 7.4 指令（`--release`，各 worktree 自己的 `CARGO_TARGET_DIR`）

啟動場景前，核對實際執行檔與啟動腳本支援的模式、載具載入方式、出生點及地形高度來源；預設或合成停機位與高度回退須明示，不能據此宣稱已驗證指定機場的停放與接地。

下列包含 P-38G 的範例命令；選用前確認測試實際載入目標載具、最新匯出包與對應參考資料。其他機型的測試通過不能證明本機型準確；沒有適用測試時，直接對目標匯出物執行量測及渲染檢查，不為了跑範例而擴大任務範圍。
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
截圖逐張目視：先確認目標機、版本、位置及視角正確，再檢查輪廓、剖面、窗框、可動件、表面貼合與材質。渲染命令成功、PNG 存在或載入測試通過都不等於視覺通過；空場景、遮擋、錯誤地形高度與近景穿透必須修正。需要場景驗證時，在指定機場核對座標、停機位及輪底對實際渲染地面的接觸，交付真實畫面並說明剩餘不確定性。修改一機的表面時，在相同相機、光照及確定性渲染條件下檢查其他機無意外變動（可用 `magick compare -metric AE`）。

---

## 8. 提交前門檻與已知陷阱

```sh
./scripts/ci.sh
```
依倉庫規則執行完整 CI；此腳本是閘門與順序的權威來源。另執行適用的模型重建、匯出後量測及目視驗證。任一必需閘門失敗都要交代實際錯誤，不能以文件修改或局部模型檢查通過宣稱整體 CI 通過。

陷阱：
- **`cargo fmt --all`** 會連同以路徑依賴的兄弟 repo 一起格式化，改到別人的工作區。
- **每個 worktree 用自己的 `CARGO_TARGET_DIR`**：共用 target 時同名 crate 互相覆寫且仍顯示 Fresh，測試與截圖會跑到別的 worktree 的程式；建置前 `df -h ~`，完成後在自己的 target `cargo clean`。
- **版本控制只處理自己擁有的變更**：遵循適用的版本控制工具流程，不在共用 checkout 重設他人的分支或未提交工作。兄弟 repo 漂移時核對正式依賴 pin，不以暫存快照或路徑覆寫繞過建置。
- **jsbsim-rs 活塞發動機在同一模型第二次 `run_ic` 後不會重新運轉**（`propulsion/set-running` 不生效）；試飛工具因此每點重新載入模型。
- **生成時的槳距**：`EngineStartMode::Running` 跑 JSBSim 的發動機穩態求解，不執行 FCS；FCS 調速器的機型必須保留內建調速器，否則停機坪上以細距起步並自行滑走。
- **JSBSim 配平**（`do_trim(Full)`）在 FCS 驅動槳角的機型上不收斂，且增壓調壓器的平坦區使油門軸導數為零。配平失敗不破壞狀態；需要穩定飛行的測試改用短時間閉環穩定。
- **JSBSim 把全富油（φ 1.3）當最佳功率混合比**，且燃油流量 = 指示功率 × `bsfc`；全富油爬升與油耗須照手冊程序飛並對照，不能直接讀。
- JSBSim 正方向舵使機首向左、正前輪轉向使機首向右；踏板到前輪的連桿增益為 −1。
- 這台機器的 `cp` 是互動式別名；腳本中覆寫檔案用 `\cp -f`。
