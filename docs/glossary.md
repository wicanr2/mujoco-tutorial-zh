# 中英術語對照表（Glossary）

依 AGENTS.md 翻譯規範：專有名詞首次出現附中英文對照；程式碼、API、XML 標籤不翻譯。

| 英文 | 繁體中文 | 備註 |
| --- | --- | --- |
| physics engine | 物理引擎 | |
| generalized coordinates | 廣義座標 | |
| contact dynamics | 接觸動力學 | |
| constraint | 約束 / 拘束 | 全專案統一用「約束」 |
| complementarity problem (LCP/NCP) | 互補問題 | |
| convex optimization | 凸最佳化 | |
| solver | 求解器 | |
| joint | 關節 | |
| body | 剛體 / body | MJCF 元素時保留 `<body>` |
| geom | 幾何體 / geom | MJCF 元素時保留 `<geom>` |
| site | 站點 / site | MJCF 元素時保留 `<site>` |
| tendon | 肌腱 | |
| actuator | 致動器 | |
| transmission | 傳動 | |
| forward dynamics | 順向動力學 | |
| inverse dynamics | 逆向動力學 | |
| model compilation | 模型編譯 | |
| degree of freedom (DOF) | 自由度 | |
| inertia | 慣性 | |
| friction cone | 摩擦錐 | |
| timestep | 時間步長 | |
| scene description language | 場景描述語言 | |
| constraint island | 約束島嶼 | |
| plugin | 外掛 | |
| visualizer / renderer | 視覺化工具 / 渲染器 | |
| mesh | 網格 | 3D 模型檔不翻譯時可用 mesh |
| keyframe | 關鍵影格 | |
| Jacobian | 雅可比矩陣 | |
| kinematic tree | 運動學樹 | |
| weld | 焊死 / 焊接 | 無 joint 的 body 被固定於父 body |
| damping | 阻尼 | |
| stiffness | 剛性 | |
| quaternion | 四元數 | |
| default settings | 預設值機制 | MJCF 的類 CSS 機制 |
| autonomous mobile robot (AMR) | 自主移動機器人 | 專案內多稱「搬運車」或「叉車」 |
| forklift | 叉車 | |
| fork / prong | 牙叉 / 叉齒 | |
| mast | 門架 | 叉車升降機構的立柱 |
| carriage | 滑架 | 沿門架升降、承載牙叉的部件 |
| mast tilt | 門架前傾 / 後傾 | 前傾卸貨、後傾靠貨 |
| sideshift | 側移 | 牙叉整組左右平移 |
| reach | 伸叉 / reach | 門架或滑台前後（X）或側向（Y）伸出 |
| stage | 門架前後移動 | MR1533 的 prismatic X 關節 |
| pallet | 棧板 | |
| stringer | 枕木 | 棧板底部縱向支撐，叉齒插入處 |
| steer wheel / swivel | 舵輪 | 可轉向的驅動輪 |
| differential drive | 差速驅動 | |
| waypoint | 路徑點 | |
| pure pursuit | 純追蹤法 | 路徑追蹤控制法 |
| gripper | 夾爪 | |
| end effector | 末端執行器 | 專案內簡稱「末端」 |
| inverse kinematics (IK) | 逆運動學 | |
| damped least squares | 阻尼最小平方法 | IK 求解方式 |
| workspace | 工作空間 | 手臂可達範圍 |
| equality constraint | 等式約束 | MJCF 的 `<equality>`，weld 屬於此類 |
| stick-slip | 黏滑 | 靜摩擦與動摩擦交替的蠕動 |
| quasi-static | 準靜態 | 慢速、慣性可忽略的過程 |
| offscreen rendering | 離屏渲染 | 無視窗環境的渲染 |
| filmstrip | 多幀圖條 | 把過程做成一排影格 |
| reinforcement learning (RL) | 強化學習 | |
| swing-up | 盪起 | 把擺從垂下盪到直立 |
| policy | 策略 | |
| reward | 回報 | |
| rollout | 軌跡取樣 | 跑一段模擬收集資料 |
| on-policy / off-policy | 同策略 / 異策略 | PPO 屬前者，SAC 屬後者 |
| sample efficiency | 樣本效率 | |
| frame skip | 跳幀 | 一個動作重複套用數個模擬步 |
| linear quadratic regulator (LQR) | 線性二次調節器 | |
| energy shaping | 能量整形 | |
