## Agent skills

### Issue tracker

Issues are tracked as local markdown files under `.scratch/<feature-slug>/`. See `docs/agents/issue-tracker.md`.

### Triage labels

Triage labels use the default vocabulary: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context layout — one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.



## **通用原则**

- 所有交流、分析、计划、代码注释、文档说明均使用中文。
- 优先保证代码正确性、可维护性和可测试性。
- 修改代码前先理解现有架构，不得随意重构无关模块。
- 所有新增功能、缺陷修复、重构均需同步更新项目文档。

------

## **Python 环境要求**

- 所有 Python 相关操作必须在项目根目录的 `venv/` 虚拟环境中执行。
- 禁止直接使用系统 Python。
- 执行 Python 命令前应先激活虚拟环境：

### **Linux/macOS**

```bash
source venv/bin/activate
```

### **Windows**

```powershell
venv\Scripts\activate
```

- 安装依赖、运行脚本、执行测试均需在虚拟环境内完成。

------

## **开发流程**

每次接收任务时按以下顺序执行：

### **1. 分析任务**

- 阅读相关代码。
- 明确需求边界。
- 评估影响范围。
- 制定实现方案。

### **2. 实施修改**

- 优先复用现有代码。
- 保持代码风格一致。
- 避免引入不必要依赖。

### **3. 更新文档**

每完成一个任务，必须检查并更新 `docs/` 目录。

要求：

- 新功能 → 新增或更新对应设计文档。
- Bug 修复 → 补充问题原因及修复说明。
- 配置变更 → 更新部署文档。
- 接口变更 → 更新接口文档。

若不存在对应文档，则创建新文档。

------

## **测试要求**

提交代码前必须完成测试。

### **Workflow 测试**

执行项目已有 Workflow：

```bash
.github/workflows/
```

对应的本地测试流程。

确保：

- 构建成功
- 单元测试通过
- 集成测试通过

### **Hooks 测试**

提交前必须验证：

```bash
.git/hooks/
```

中的 Hook 能够正常执行。

包括但不限于：

- pre-commit
- pre-push

禁止绕过 Hook。

------

## **Git 提交规范**

完成开发后必须提交 Git Commit。

Commit Message 使用以下格式：

### **新功能**

```text
feature: 功能描述
```

例如：

```text
feature: 新增无人机任务调度模块
```

### **缺陷修复**

```text
fix: 问题描述
```

例如：

```text
fix: 修复地图同步状态异常
```

### **文档更新**

```text
docs: 更新内容
```

例如：

```text
docs: 补充部署说明
```

### **重构**

```text
refactor: 重构内容
```

### **性能优化**

```text
perf: 优化内容
```

### **测试相关**

```text
test: 测试内容
```

------

## **提交前检查清单**

提交前必须确认：

- 已使用中文完成所有交流
- 已完成需求实现
- 已更新 docs/ 文档
- 已在 venv 环境中执行相关 Python 操作
- 已通过 Workflow 测试
- 已通过 Hooks 测试
- 已完成 Git Commit
- Commit Message 符合规范

若上述任一项未完成，则禁止结束任务。
