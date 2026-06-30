# Custom Agent Rules for sim25 Workspace

## 빌드 규칙 (Build Rules)
- **빌드 작업은 사용자가 직접 수행합니다.**
- AI 에이전트는 소스코드 컴파일, MSBuild 실행, `buildue.bat` 실행 등 프로젝트나 언리얼 엔진의 빌드를 직접 수행하지 마십시오.
- **단, `sil_app` (x_memory2 폴더 안의 `build.bat` 실행 등) 빌드는 예외적으로 AI 에이전트가 자동으로 빌드할 수 있습니다.**
- 빌드가 필요한 경우 빌드 스크립트의 작성/수정까지만 지원하고, 실제 컴파일 및 빌드는 사용자에게 직접 실행하도록 안내하십시오. (단, sil_app은 제외)
