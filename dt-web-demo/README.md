# 低空数字孪生 Web Demo

中心 DT 与区域 DT 的前端演示代码，支持桌面交互和 16:9 评审展示。

## 依赖

- 浏览器：新版 Chrome 或 Edge，支持 Canvas、Web Worker。
- 启动服务：Python 3（仅使用内置模块，无需安装 Python 包），或自行提供静态 HTTP 服务。
- 页面由 HTML、CSS 和原生 JavaScript 构成，无需 npm install、构建、数据库或后端服务，也不依赖外部 CDN。
- Node.js、Playwright 仅用于开发测试，运行页面不需要。

## 本地运行

在准备存放代码的目录中执行：

```powershell
git clone https://github.com/Linkaigithub/dt-web-demo.git
cd dt-web-demo
python -m http.server 8767 --bind 127.0.0.1 --directory web-demo
```

浏览器打开 **http://127.0.0.1:8767/**。按 `Ctrl+C` 停止服务。若端口被占用，将命令和访问地址中的 `8767` 同时替换。

请通过 HTTP 服务访问，以确保 Web Worker 正常加载；不推荐直接双击 HTML。服务只监听本机。如使用其他静态服务，将网站根目录指向 `web-demo/`，并保留全部脚本的相对路径。

## 如何使用

1. 默认进入中心 DT，可选择场景、区域和方向层，查看态势、历史曲线和策略对比。
2. 点击“查看区域”进入区域 DT；可旋转、缩放场景，点击对象或搜索 ID 查看详情。
3. 点击“切换到16:9全览”进入评审视图；全览中的“查看区域”和“查看中心”保持 16:9 模式。按 `Esc` 返回详情视图；图表放大时先退出图表放大。
4. 选择推演时域和策略，点击“从当前全域推演”或“从当前区域推演”，生成策略记录；使用时间轴播放或回看。

面向电脑端；目标显示分辨率为 1920×1080、2560×1440、3840×2160。首次进入历史对比时会在后台计算数据。

## 后端接入说明

当前为离线模拟演示：数值、需求、个体轨迹及推演结果均在浏览器生成，尚未调用真实后端接口。历史观测使用模拟替代值，页面结果不代表实际运行或算法验收结论。

- 页面入口：`web-demo/index.html`。
- 中心：`center.js` / `center.css`；模型和推演：`center-model.js`、`center-projection.js`。
- 区域：`regional.js` / `regional.css`；推演：`branch-simulation.js`、`region-projection.js`。
- 历史计算：`center-history*.js`、`region-history*.js`；评价：`region-evaluation.js`。
- 数据口径及接入参考：[区域 DT 评估与数据对接](web-demo/docs/区域DT评估与数据对接.md)。该文档是设计参考，实际接口需双方另行约定。

接入时需对齐区域/设施 ID、拓扑、时间戳、指标单位和缺失值语义，并替换对应的模拟数据入口。

## 目录与测试

```text
web-demo/
  index.html       页面入口
  *.js / *.css     界面、模型、推演和后台计算
  docs/            设计、指标口径和历次验证记录
  tests/           模型与浏览器测试
  review/          历史截图
```

安装 Node.js 后，可在仓库根目录运行无第三方依赖的模型测试：

```powershell
node web-demo/tests/verify-branches.cjs
node web-demo/tests/verify-center-projection.cjs
```

浏览器测试另需 Playwright 和匹配的浏览器；部分历史回归还依赖原项目的基线或备份目录，不能作为独立仓库开箱即用的测试。`web-demo/serve-review.cjs` 是原项目双版本对照工具，不作为本仓库的推荐启动方式。`web-demo/README.md` 及 `docs/` 含历次记录；本仓库依赖与启动以本页为准。

## 版本来源

首次导出于 2026-11-06，来源为原项目 `main` 的 `b6d1dee5f1b76c605f775fd7f9143a3645b24bba`，仅包含该提交的 `web-demo/` 内容，以及本仓库新增的 README 和忽略规则；不包含原项目 Git 历史或未提交修改。

本仓库与原项目独立，后续更新需要显式同步。