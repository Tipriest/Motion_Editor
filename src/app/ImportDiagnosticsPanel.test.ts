import { readFileSync } from 'node:fs';
import { JSDOM } from 'jsdom';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { ImportDiagnosticsPanel } from './ImportDiagnosticsPanel';

const html = readFileSync(new URL('../../index.html', import.meta.url), 'utf8');
let dom: JSDOM;
let panel: ImportDiagnosticsPanel;
let button: HTMLButtonElement;
let dialog: HTMLDialogElement;

beforeEach(() => {
  dom = new JSDOM(html);
  const document = dom.window.document;
  button = document.querySelector('#import-check-btn')!;
  dialog = document.querySelector('#import-check-dialog')!;
  // jsdom does not implement native modal rendering. Keep real DOM event and text behavior.
  dialog.showModal = vi.fn(() => { dialog.setAttribute('open', ''); });
  dialog.close = vi.fn(() => { dialog.removeAttribute('open'); });
  panel = new ImportDiagnosticsPanel(document);
});

afterEach(() => {
  panel.dispose();
  dom.window.close();
});

describe('data inspection button and dialog', () => {
  it('is disabled before an import and becomes a clickable success button', async () => {
    expect(button.disabled).toBe(true);
    expect(button.dataset.status).toBe('idle');
    await panel.diagnostics.run('motion.csv', async () => {
      expect(button.disabled).toBe(true);
      expect(button.textContent).toContain('导入中');
      panel.diagnostics.observeState('playing', {});
    }, () => ({ rows: [['fps', '50']], warnings: [] }));
    expect(button.dataset.status).toBe('success');
    expect(button.disabled).toBe(false);
    button.click();
    expect(panel.isOpen).toBe(true);
    expect(dialog.textContent).toContain('当前检查未发现问题');
    expect(dialog.querySelector('dl')!.textContent).toContain('fps50');
    expect(dialog.querySelector('#import-check-issues')!.hasAttribute('hidden')).toBe(true);
    (dialog.querySelector('#import-check-close-btn') as HTMLButtonElement).click();
    expect(panel.isOpen).toBe(false);
  });

  it('shows successful imports with warnings and preserves literal filenames and parser text', async () => {
    const source = '<img src=x onerror=alert(1)>.pkl';
    const warning = 'Missing <root_pos>; used default & verify';
    await panel.diagnostics.run(source, async () => {
      panel.diagnostics.observeState('playing', { warnings: [warning] });
    }, () => ({ rows: [], warnings: [warning] }));
    expect(button.dataset.status).toBe('success');
    expect(button.textContent).toContain('1 条提示');
    button.click();
    expect(dialog.textContent).toContain(source);
    expect(dialog.textContent).toContain(warning);
    expect(dialog.querySelector('img')).toBeNull();
    expect(dialog.querySelector('[data-severity="warning"]')).not.toBeNull();
  });

  it('keeps failures clickable after viewer recovery and updates an open report on reimport', async () => {
    await panel.diagnostics.run('broken.pkl', async () => {
      panel.diagnostics.observeState('error', { detail: 'GMR missing root_pos' });
    }, () => ({ rows: [], warnings: [] }));
    expect(button.dataset.status).toBe('error');
    expect(button.disabled).toBe(false);
    panel.diagnostics.observeState('model_ready', {});
    button.click();
    expect(dialog.textContent).toContain('GMR missing root_pos');
    await panel.diagnostics.run('fixed.pkl', async () => {
      expect(dialog.textContent).not.toContain('GMR missing root_pos');
      panel.diagnostics.observeState('playing', {});
    }, () => ({ rows: [], warnings: [] }));
    expect(dialog.textContent).toContain('fixed.pkl');
    expect(button.dataset.status).toBe('success');
    panel.diagnostics.reset();
    expect(panel.isOpen).toBe(false);
    expect(button.disabled).toBe(true);
  });

  it('closes on backdrop clicks while preserving clicks in the dialog content', async () => {
    await panel.diagnostics.run('bad.csv', async () => { throw new Error('bad data'); }, () => ({ rows: [], warnings: [] }));
    button.click();
    dialog.getBoundingClientRect = () => ({ left: 10, right: 100, top: 10, bottom: 100 } as DOMRect);
    dialog.dispatchEvent(new dom.window.MouseEvent('click', { clientX: 50, clientY: 50 }));
    expect(panel.isOpen).toBe(true);
    dialog.dispatchEvent(new dom.window.MouseEvent('click', { clientX: 200, clientY: 200 }));
    expect(panel.isOpen).toBe(false);
    panel.dispose();
    button.click();
    expect(panel.isOpen).toBe(false);
  });
});
