# AIC Competition Source

大模型技术创新赛源码仓库，保留源码版本历史与回退点。

## API 配置

仓库提供 `src/yzbot/aic_task/config/api_secrets.example.yaml`，其中 `deepseek.api_key` 留空。使用者需要填写自己的 DeepSeek API Key。

在仓库根目录执行（仅首次配置，已有配置不要覆盖）：

```bash
cp -n src/yzbot/aic_task/config/api_secrets.example.yaml src/yzbot/aic_task/config/api_secrets.yaml
chmod 600 src/yzbot/aic_task/config/api_secrets.yaml
```

编辑生成的 `api_secrets.yaml`，将 `deepseek.api_key` 的空字符串替换为自己的密钥。其余连接设置可按需要调整。未填写有效密钥时，API 功能无法正常使用。

程序读取 `api_secrets.yaml`；`.example.yaml` 是分发模板。实际配置文件已被 `.gitignore` 排除，不会随提交或推送上传。不要强制加入 Git，也不要把密钥写进文档、日志或提交记录。Git 回退不会恢复本机密钥，换电脑时需要重新配置。

从 Git 生成独立运行快照时，还需单独复制本机配置到快照对应的 `src/yzbot/aic_task/config/api_secrets.yaml`，并保持权限 `600`。

## 版本与云端

私人仓库：https://github.com/imsquner/AIC_Competition_Source

本地修改经验证后提交，再通过 `git push origin main` 同步云端。回退已提交变更优先使用 `git revert <commit>`，保留版本历史。私密配置、构建产物和运行日志不加入 Git。

本机路径、构建与运行说明见 [README_LOCAL.md](README_LOCAL.md)。
