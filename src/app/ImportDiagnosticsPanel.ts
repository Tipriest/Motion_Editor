import { ImportDiagnostics } from './ImportDiagnostics';

export class ImportDiagnosticsPanel {
  readonly diagnostics = new ImportDiagnostics(() => this.render());
  private readonly button: HTMLButtonElement;
  private readonly dialog: HTMLDialogElement;
  private readonly closeButton: HTMLButtonElement;
  private readonly summary: HTMLElement;
  private readonly details: HTMLElement;
  private readonly issues: HTMLElement;

  constructor(root: ParentNode = document) {
    const require = <T extends HTMLElement>(id: string): T => {
      const element = root.querySelector<T>(`#${id}`);
      if (!element) throw new Error(`Required element not found: #${id}`);
      return element;
    };
    this.button = require('import-check-btn');
    this.dialog = require('import-check-dialog');
    this.closeButton = require('import-check-close-btn');
    this.summary = require('import-check-summary');
    this.details = require('import-check-details');
    this.issues = require('import-check-issues');
    this.button.addEventListener('click', this.open);
    this.closeButton.addEventListener('click', this.close);
    this.dialog.addEventListener('click', this.onBackdropClick);
    this.render();
  }

  get isOpen(): boolean { return this.dialog.open; }

  private readonly open = (): void => {
    if (!this.button.disabled && !this.dialog.open) this.dialog.showModal();
  };
  private readonly close = (): void => { this.dialog.close(); };
  private readonly onBackdropClick = (event: MouseEvent): void => {
    if (event.target !== this.dialog) return;
    const bounds = this.dialog.getBoundingClientRect();
    if (event.clientX < bounds.left || event.clientX > bounds.right ||
        event.clientY < bounds.top || event.clientY > bounds.bottom) this.close();
  };

  private render(): void {
    const report = this.diagnostics.report;
    const issueCount = report.errors.length + report.warnings.length;
    this.button.dataset.status = report.status;
    this.button.disabled = report.status === 'idle' || report.status === 'loading';
    const label = report.status === 'loading' ? '数据检查 · 导入中' :
      report.status === 'error' ? '数据检查 · 导入失败' :
      report.status === 'success' ? `数据检查 · 导入成功${issueCount ? ` · ${issueCount} 条提示` : ''}` : '数据检查';
    this.button.textContent = label;
    this.button.title = label;
    this.summary.textContent = report.status === 'error'
      ? '导入失败。下面列出解析器返回的原因和处理提示。'
      : report.status === 'loading' ? '正在解析本次输入…' : report.status === 'idle' ? '尚无导入结果。'
      : issueCount ? '导入成功，请检查下面的提示。' : '导入成功，当前检查未发现问题。';
    this.details.replaceChildren();
    const rows: Array<[string, string]> = [['本次输入', report.source], ...report.rows];
    for (const [label, value] of rows) {
      const term = this.details.ownerDocument.createElement('dt');
      const description = this.details.ownerDocument.createElement('dd');
      term.textContent = label;
      description.textContent = value;
      this.details.append(term, description);
    }
    this.issues.replaceChildren();
    for (const [severity, messages] of [['error', report.errors], ['warning', report.warnings]] as const) {
      for (const message of messages) {
        const item = this.issues.ownerDocument.createElement('li');
        item.dataset.severity = severity;
        item.textContent = `${severity === 'error' ? '错误' : '提示'}：${message}`;
        this.issues.append(item);
      }
    }
    this.issues.hidden = issueCount === 0;
    if (report.status === 'idle') this.close();
  }

  dispose(): void {
    this.close();
    this.button.removeEventListener('click', this.open);
    this.closeButton.removeEventListener('click', this.close);
    this.dialog.removeEventListener('click', this.onBackdropClick);
  }
}
