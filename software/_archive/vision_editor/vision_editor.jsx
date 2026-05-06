/* eslint-disable */
// vision_editor.jsx — React Flow node editor for the holOS vision pipelines.
//
// Loaded as a JSX file; transpiled in-browser by Babel-standalone (referenced
// from index.html). Self-contained: no build step, no module resolution.
//
// Talks to the Flask backend via:
//   GET    /api/vision/pipelines           → list pipelines + node kinds
//   PUT    /api/vision/pipelines/<name>    → create / replace a graph
//   DELETE /api/vision/pipelines/<name>    → remove
//   POST   /api/vision/pipelines/<name>/enable  { enabled: bool }
//
// And subscribes to SocketIO event `vision_feed` for live JPEG dashboard tiles.
//
// `socket` is the global socket.io client created in index.html.

const { useState, useEffect, useCallback, useRef, useMemo } = React;
const RF = window.ReactFlow;
const ReactFlow         = RF.default || RF.ReactFlow;
const ReactFlowProvider = RF.ReactFlowProvider;
const Background        = RF.Background;
const Controls          = RF.Controls;
const Handle            = RF.Handle;
const Position          = RF.Position;
const useNodesState     = RF.useNodesState;
const useEdgesState     = RF.useEdgesState;
const addEdge           = RF.addEdge;

// ── Port-kind → wire color (informational) ────────────────────────────────
// frame    = clean processed image (sky-blue)
// preview  = annotated/overlay image — for display only (amber)
// Both render visually as images; the kind separation prevents wiring an
// already-annotated preview into a downstream processing node by accident.
const KIND_COLOR = {
  frame:     '#38bdf8',
  preview:   '#fbbf24',
  detection: '#f59e0b',
  pose_list: '#a78bfa',
  mask:      '#f472b6',
  json:      '#94a3b8',
  pose:      '#34d399',
};

// Connect rule: which output kind can feed which input kind.
// Same-kind always wired through; the only relaxation is FRAME → PREVIEW
// (a clean frame is a valid backdrop for a display node, but a PREVIEW
// must NOT flow back into something that processes/distorts).
function isConnectionValid(srcKind, dstKind) {
  if (!srcKind || !dstKind) return true;       // unknown kinds = allow
  if (srcKind === dstKind) return true;
  if (srcKind === 'frame' && dstKind === 'preview') return true;
  return false;
}

// Inline playback strip rendered on source.* nodes in the editor.
// Mirrors the dashboard's per-feed strip — same actions, same styling.
function NodePlaybackStrip({ kind, params, onParamChange }) {
  const mode  = params.playback || (kind === 'source.camera' ? 'live' : 'play');
  const speed = parseFloat(params.speed ?? 1.0);

  const fire = (action) => {
    if (kind === 'source.video') {
      if (action === 'step_back_10' || action === 'step_10') {
        // Editor doesn't have live frame_idx — fall back to a 10-frame
        // jump via seek_target relative to current.
        const cur = params.seek_target ?? 0;
        const delta = action === 'step_back_10' ? -10 : 10;
        onParamChange('playback', 'seek');
        onParamChange('seek_target', Math.max(0, cur + delta));
      } else {
        onParamChange('playback', action);
      }
    } else if (kind === 'source.camera') {
      onParamChange('playback', action);
    } else if (kind === 'source.image') {
      if (action === 'refresh') {
        onParamChange('playback', 'once');
        onParamChange('refresh', (params.refresh ?? 0) + 1);
      } else {
        onParamChange('playback', action);
      }
    }
  };

  const Btn = ({ act, label, title, active }) => (
    <button className={'vne-pb-btn' + (active ? ' vne-pb-active' : '')}
            onClick={(e) => { e.stopPropagation(); fire(act); }}
            title={title || act}>{label}</button>
  );

  if (kind === 'source.video') {
    return (
      <div className="vne-pb-strip" onClick={(e) => e.stopPropagation()}>
        <Btn act="step_back_10" label="⏮⏮" title="-10 frames" />
        <Btn act="step_back"    label="⏮"   title="-1 frame" />
        <Btn act="pause"        label="⏸"   active={mode === 'pause'} />
        <Btn act="play"         label="▶"   active={mode === 'play'} />
        <Btn act="step"         label="⏭"   title="+1 frame" />
        <Btn act="step_10"      label="⏭⏭"  title="+10 frames" />
        <select className="vne-pb-speed" value={speed}
                onChange={(e) => { e.stopPropagation();
                  onParamChange('speed', parseFloat(e.target.value)); }}
                title="Playback speed">
          {[0.25, 0.5, 1, 2, 4, 8].map(s =>
            <option key={s} value={s}>{s}×</option>)}
        </select>
      </div>
    );
  }
  if (kind === 'source.camera') {
    return (
      <div className="vne-pb-strip" onClick={(e) => e.stopPropagation()}>
        <Btn act="live"  label="● live" active={mode === 'live'} />
        <Btn act="pause" label="⏸"      active={mode === 'pause'} />
        <Btn act="step"  label="⏭"      title="capture next frame" />
      </div>
    );
  }
  if (kind === 'source.image') {
    const isOnce = (mode === 'once');
    return (
      <div className="vne-pb-strip" onClick={(e) => e.stopPropagation()}>
        <Btn act={isOnce ? 'every_tick' : 'once'}
             label={isOnce ? 'once' : '↻ tick'} active />
        <Btn act="refresh" label="⟳" title="emit one new frame" />
      </div>
    );
  }
  return null;
}

// ── Generic node renderer — driven by NodeKindInfo.inputs/outputs ─────────
//
// Each row contains BOTH an input (left) and an output (right) port if both
// exist for that index. We measure each row's actual offset (with respect
// to the node container) at layout time and pass that as the Handle's
// `top` style. This makes the dot-alignment immune to header / font /
// padding changes — it just works.
const PORT_ROW_H = 22;

function GenericNode({ data, id }) {
  const { kind, label, info, params, onParamChange, state, preview } = data;
  const inputs  = info?.inputs  || [];
  const outputs = info?.outputs || [];
  const schema  = info?.params_schema || {};
  const portRows = Math.max(inputs.length, outputs.length);
  const rowRefs  = useRef([]);
  const [handleTops, setHandleTops] = useState([]);

  // After every render, measure each port-row's vertical center relative
  // to the node container. Re-runs when the row count changes.
  React.useLayoutEffect(() => {
    if (!rowRefs.current.length) {
      setHandleTops([]);
      return;
    }
    const tops = rowRefs.current.map(el => {
      if (!el) return 0;
      // offsetTop is relative to the closest positioned ancestor — the
      // node wrapper (RF puts position:relative on every node).
      const node = el.closest('.react-flow__node') || el.closest('.vne-node');
      const nodeRect = node?.getBoundingClientRect();
      const rowRect  = el.getBoundingClientRect();
      if (!nodeRect) return el.offsetTop + el.offsetHeight / 2;
      return (rowRect.top - nodeRect.top) + (rowRect.height / 2);
    });
    setHandleTops(tops);
  }, [portRows]);

  return (
    <div className="vne-node">
      <div className="vne-node-header">
        <span className="vne-node-title">{label || id}</span>
        <span className="vne-node-kind">{kind}</span>
      </div>

      <div className="vne-node-ports">
        {Array.from({ length: portRows }).map((_, idx) => {
          const inP  = inputs[idx];
          const outP = outputs[idx];
          const top  = handleTops[idx] ?? null;
          return (
            <div key={'row-' + idx}
                 ref={el => { rowRefs.current[idx] = el; }}
                 className="vne-port-row"
                 style={{ height: PORT_ROW_H }}>
              {inP && (
                <>
                  <Handle
                    type="target"
                    id={inP.name}
                    position={Position.Left}
                    className={inP.optional ? 'vne-handle-optional' : ''}
                    style={{
                      // Optional ports render hollow (transparent fill +
                      // colored border) so the user immediately sees
                      // they're not required.
                      background: inP.optional
                                  ? 'transparent'
                                  : (KIND_COLOR[inP.kind] || '#888'),
                      borderColor: KIND_COLOR[inP.kind] || '#888',
                      ...(top != null ? { top } : {}),
                    }}
                    title={inP.optional ? `${inP.name} (optional)` : inP.name}
                  />
                  <span className={'vne-port-label vne-port-label-in' + (inP.optional ? ' vne-port-optional' : '')}>
                    {inP.name}{inP.optional && <em>?</em>}
                  </span>
                </>
              )}
              <span style={{ flex: 1 }} />
              {outP && (
                <>
                  <span className="vne-port-label vne-port-label-out">{outP.name}</span>
                  <Handle
                    type="source"
                    id={outP.name}
                    position={Position.Right}
                    style={{
                      background: KIND_COLOR[outP.kind] || '#888',
                      ...(top != null ? { top } : {}),
                    }}
                  />
                </>
              )}
            </div>
          );
        })}
      </div>

      <div className="vne-node-body">
        {/* Param form. Note: playback / speed controls for source nodes
            are deliberately NOT rendered here — they live on the
            dashboard tiles. The graph editor stays focused on wiring
            and configuration. */}
        {Object.keys(schema).length > 0 && (
          <details className="vne-params" onClick={(e) => e.stopPropagation()}>
            <summary>params</summary>
            {Object.entries(schema).map(([k, def]) => (
              <ParamField key={k} k={k} def={def}
                          value={params[k] ?? def.default}
                          onChange={(v) => onParamChange(id, k, v)} />
            ))}
          </details>
        )}

        {/* Inline preview — the most-recent frame the freeze-frame debug
            run produced for this node. Lives entirely in the editor and
            is NOT the same as the `frame` output port (that's the live
            value that flows downstream). */}
        {preview && (
          <div className="vne-node-preview">
            <div className="vne-node-preview-label">preview</div>
            <img src={`data:image/jpeg;base64,${preview}`} alt="node preview" />
          </div>
        )}

        {/* Inline state badges */}
        {state?.last_error && (
          <div className="vne-state-err">⚠ {state.last_error}</div>
        )}
      </div>
    </div>
  );
}

// Cache for the file-picker dropdowns: kind ('video' | 'image' | 'all') → array.
const _filePickerCache = {};
async function _fetchFiles(kind) {
  if (_filePickerCache[kind]) return _filePickerCache[kind];
  try {
    const r = await fetch(`/api/vision/list_files?kind=${kind}`);
    const data = await r.json();
    _filePickerCache[kind] = data.files || [];
    return _filePickerCache[kind];
  } catch (_) { return []; }
}

function FilePickerField({ k, def, value, onChange }) {
  const kind = def.kind || 'all';
  const [browseOpen, setBrowseOpen] = useState(false);
  return (
    <label className="vne-pf">
      <span>{k}</span>
      <input type="text" className="input-sm" value={value || ''}
             style={{ flex: 1, minWidth: 0 }}
             placeholder="path to file"
             onChange={e => onChange(e.target.value)} />
      <button className="btn-tiny" onClick={(e) => { e.preventDefault(); setBrowseOpen(true); }}
              title="Browse for a file">📂</button>
      {browseOpen && (
        <FileBrowserModal
          kind={kind}
          /* Open at the parent directory of the current value. The backend
             also falls back to software/vision if the path is a file or
             unresolvable, but trimming here means the modal lands at the
             right folder visually rather than flashing 'not a directory'. */
          startPath={(() => {
            if (!value) return 'software/vision';
            // Drop the trailing filename (everything after the last / or \)
            const trimmed = String(value).replace(/\\/g, '/');
            const slash = trimmed.lastIndexOf('/');
            const looksLikeFile = /\.[a-z0-9]{2,5}$/i.test(trimmed);
            return looksLikeFile && slash > 0
              ? trimmed.slice(0, slash)
              : trimmed;
          })()}
          onClose={() => setBrowseOpen(false)}
          onPick={(path) => { onChange(path); setBrowseOpen(false); }} />
      )}
    </label>
  );
}

// ── Server-side directory browser modal ───────────────────────────────────
// Browsers can't expose absolute paths from a native <input type=file>, so
// we list a sandboxed view of the project tree and let the user navigate +
// pick a file. Returns a path relative to the project root, which the
// backend resolves the same way as a typed path.
function FileBrowserModal({ kind, startPath, onClose, onPick }) {
  const [cwd, setCwd] = useState(startPath || 'software/vision');
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(true);
  const [err, setErr] = useState(null);

  const load = async (path) => {
    setLoading(true); setErr(null);
    try {
      const r = await fetch(
        `/api/vision/browse?path=${encodeURIComponent(path)}&kind=${encodeURIComponent(kind)}`
      );
      const d = await r.json();
      if (!d.ok) { setErr(d.error || 'browse failed'); }
      else { setData(d); setCwd(d.cwd); }
    } catch (e) { setErr(String(e)); }
    finally { setLoading(false); }
  };

  useEffect(() => { load(startPath || 'software/vision'); }, []);

  const stop = (e) => e.stopPropagation();
  return (
    <div className="vne-modal-backdrop" onClick={onClose}>
      <div className="vne-modal" onClick={stop}>
        <div className="vne-modal-hdr">
          <span style={{ fontWeight: 600 }}>Browse — {kind}</span>
          <button className="btn-tiny" onClick={onClose} title="Close">✕</button>
        </div>
        <div className="vne-modal-cwd" title={data?.project_root}>
          📁 <code>{cwd || '/'}</code>
        </div>
        <div className="vne-modal-body">
          {loading && <div style={{ color: 'var(--text-dim)' }}>loading…</div>}
          {err && <div style={{ color: '#fca5a5' }}>error: {err}</div>}
          {!loading && !err && data && (
            <ul className="vne-fb-list">
              {cwd && (
                <li className="vne-fb-row vne-fb-up"
                    onClick={() => load(data.parent || '')}>
                  ⬑  ..
                </li>
              )}
              {(data.entries || []).map(e => (
                <li key={e.rel}
                    className={'vne-fb-row ' + (e.is_dir ? 'vne-fb-dir' : 'vne-fb-file')}
                    onDoubleClick={() => e.is_dir ? load(e.rel) : onPick(e.rel)}
                    onClick={() => e.is_dir ? load(e.rel) : null}>
                  <span className="vne-fb-icon">{e.is_dir ? '📁' : '🎬'}</span>
                  <span className="vne-fb-name">{e.name}</span>
                  {!e.is_dir && (
                    <span className="vne-fb-size">{e.size_mb} MB</span>
                  )}
                </li>
              ))}
              {!data.entries?.length && (
                <li style={{ color: 'var(--text-dim)', padding: '8px' }}>
                  (empty)
                </li>
              )}
            </ul>
          )}
        </div>
        <div className="vne-modal-ftr">
          <span style={{ flex: 1, color: 'var(--text-dim)', fontSize: 10 }}>
            Double-click a file to pick it · single-click a folder to enter it
          </span>
          <button className="btn-tiny" onClick={onClose}>Cancel</button>
        </div>
      </div>
    </div>
  );
}

function ParamField({ k, def, value, onChange }) {
  const t = def.type || 'str';
  // Display label — fall back to the raw key. Description shown as tooltip.
  const label = def.label || k;
  const tip   = def.description || def.help || '';
  const unit  = def.unit ? ` ${def.unit}` : '';

  // Common <span> for the field's left-side label + tooltip
  const Lbl = () => <span title={tip}>{label}{unit && <em className="vne-pf-unit"> ({def.unit})</em>}</span>;

  if (t === 'file_picker') {
    return <FilePickerField k={k} def={def} value={value} onChange={onChange} />;
  }
  if (def.enum) {
    return (
      <label className="vne-pf" title={tip}>
        <Lbl />
        <select className="input-sm" value={value ?? ''}
                onChange={e => onChange(e.target.value)}>
          {def.enum.map(v => <option key={v} value={v}>{v}</option>)}
        </select>
      </label>
    );
  }
  if (t === 'bool') {
    return (
      <label className="vne-pf vne-pf-bool" title={tip}>
        <input type="checkbox" checked={!!value}
               onChange={e => onChange(e.target.checked)} />
        <span>{label}</span>
      </label>
    );
  }
  if (t === 'int' || t === 'float') {
    const step = def.step ?? (t === 'float' ? 0.1 : 1);
    const min  = def.min  ?? undefined;
    const max  = def.max  ?? undefined;
    return (
      <label className="vne-pf" title={tip}>
        <Lbl />
        <input type="number" className="input-sm" value={value ?? ''}
               step={step} min={min} max={max}
               onChange={e => {
                 const v = e.target.value;
                 if (v === '') return onChange(null);
                 onChange(t === 'float' ? parseFloat(v) : parseInt(v, 10));
               }} />
      </label>
    );
  }
  if (t === 'dict' || t === 'json') {
    // Read-only summary for now — these structures don't have a generic
    // editor. (e.g., the `anchors` map; edit via the saved JSON if needed.)
    let display = '—';
    try { display = JSON.stringify(value); } catch (_) {}
    return (
      <label className="vne-pf" title={tip}>
        <Lbl />
        <code className="vne-pf-readonly">
          {display && display.length > 40 ? display.slice(0, 40) + '…' : display}
        </code>
      </label>
    );
  }
  return (
    <label className="vne-pf" title={tip}>
      <Lbl />
      <input type="text" className="input-sm" value={value ?? ''}
             onChange={e => onChange(e.target.value)} />
    </label>
  );
}

const NODE_TYPES = { generic: GenericNode };

// ── Editor root ────────────────────────────────────────────────────────────
function PipelineEditor() {
  const [registry, setRegistry] = useState(null);   // backend response
  const [pipelineName, setPipelineName] = useState(null);
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const [feeds, setFeeds] = useState({});             // feed_id → { jpeg, label }
  const [dirty, setDirty] = useState(false);

  // ── History (undo/redo) ────────────────────────────────────────────
  const historyRef = useRef({ past: [], future: [] });
  // Whenever a real edit happens we push the PRE-edit snapshot onto the
  // past stack and clear the redo stack. Calls to applyHistory()
  // (e.g. undo / redo / pipeline switch) bypass this so they don't
  // recursively pollute the history.
  const skipHistoryRef = useRef(false);

  // Take a deep-enough copy of the current graph (nodes + edges) for the
  // undo/redo stack. Renamed from `snapshot` to avoid clashing with the
  // freeze-frame state that's also called snapshot below.
  const takeHistorySnap = () => ({
    nodes: nodes.map(n => ({ ...n, data: { ...n.data, params: { ...n.data.params } } })),
    edges: edges.map(e => ({ ...e })),
  });
  const pushHistory = () => {
    if (skipHistoryRef.current) return;
    historyRef.current.past.push(takeHistorySnap());
    if (historyRef.current.past.length > 50) historyRef.current.past.shift();
    historyRef.current.future = [];
  };
  const applyHistory = (snap) => {
    skipHistoryRef.current = true;
    setNodes(snap.nodes);
    setEdges(snap.edges);
    // microtask-y reset
    setTimeout(() => { skipHistoryRef.current = false; }, 0);
  };
  const undo = () => {
    const h = historyRef.current;
    if (!h.past.length) return;
    h.future.push(takeHistorySnap());
    applyHistory(h.past.pop());
    setDirty(true);
  };
  const redo = () => {
    const h = historyRef.current;
    if (!h.future.length) return;
    h.past.push(takeHistorySnap());
    applyHistory(h.future.pop());
    setDirty(true);
  };

  // ── Freeze-frame debug run ────────────────────────────────────────
  // When the editor tab opens we grab one frame from whatever source the
  // active pipeline is running, then run THIS editor's draft graph against
  // that frozen frame. Each node renders its main frame output inline.
  const [snapshot, setSnapshot] = useState(null);  // base64 jpeg
  const [snapshotInfo, setSnapshotInfo] = useState(null);
  const debugTimerRef = useRef(null);

  // Pause every source.video node in the named pipeline (so the dashboard
  // doesn't keep advancing while we're working on the graph). Returns the
  // list of node ids that were paused.
  const pauseAllVideoSources = async (pipelineName) => {
    if (!pipelineName) return [];
    const fresh = await (await fetch('/api/vision/pipelines')).json();
    const p = (fresh.pipelines || []).find(x => x.name === pipelineName);
    if (!p) return [];
    const paused = [];
    for (const nd of (p.graph?.nodes || [])) {
      if (nd.kind !== 'source.video') continue;
      try {
        await fetch(`/api/vision/pipelines/${encodeURIComponent(pipelineName)}/node/${encodeURIComponent(nd.id)}/params`, {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ params: { playback: 'pause' } }),
        });
        paused.push(nd.id);
      } catch (_) {}
    }
    return paused;
  };

  const captureSnapshot = async ({ pauseFirst = false } = {}) => {
    const sourceName = (registry && registry.active) || pipelineName;
    if (!sourceName) {
      setSnapshotInfo({ error: 'no active pipeline to snapshot from' });
      return;
    }
    if (pauseFirst) {
      try { await pauseAllVideoSources(sourceName); } catch (_) {}
    }
    try {
      const r = await fetch(`/api/vision/snapshot/${encodeURIComponent(sourceName)}`);
      const data = await r.json();
      if (!data.ok) {
        setSnapshotInfo({ error: data.error || 'snapshot failed' });
        return;
      }
      setSnapshot(data.jpeg);
      setSnapshotInfo({ source_node: data.source_node, shape: data.shape });
    } catch (e) {
      setSnapshotInfo({ error: String(e) });
    }
  };

  const runDebug = async () => {
    if (!snapshot) return;
    const graph = exportGraph();
    if (!graph.name) graph.name = '__debug__';
    try {
      const r = await fetch('/api/vision/debug_run', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ graph, snapshot_jpeg: snapshot }),
      });
      const data = await r.json();
      if (!data.ok) return;
      // Map per-node main frames back into nodes
      setNodes(curr => curr.map(n => {
        const fr = data.frames?.[n.id]?.main?.jpeg;
        const err = data.errors?.[n.id];
        return {
          ...n,
          data: { ...n.data,
                  preview: fr || null,
                  state: { ...(n.data.state || {}),
                           last_error: err || (n.data.state || {}).last_error },
                },
        };
      }));
    } catch (e) { /* ignore */ }
  };

  // Auto-run the debug pipeline ~300ms after any structural change, so
  // adding a node or rewiring an edge updates the per-node previews.
  useEffect(() => {
    if (!snapshot) return;
    if (debugTimerRef.current) clearTimeout(debugTimerRef.current);
    debugTimerRef.current = setTimeout(runDebug, 300);
    return () => { if (debugTimerRef.current) clearTimeout(debugTimerRef.current); };
  }, [nodes.length, edges.length, snapshot]);

  // ── Clipboard ──────────────────────────────────────────────────────
  const clipboardRef = useRef(null);
  const copySelection = () => {
    const selN = nodes.filter(n => n.selected);
    if (selN.length === 0) return;
    const ids = new Set(selN.map(n => n.id));
    const selE = edges.filter(e => e.selected || (ids.has(e.source) && ids.has(e.target)));
    clipboardRef.current = {
      nodes: selN.map(n => ({ ...n, data: { ...n.data, params: { ...n.data.params } } })),
      edges: selE.map(e => ({ ...e })),
    };
  };
  const pasteSelection = () => {
    const cb = clipboardRef.current;
    if (!cb) return;
    pushHistory();
    // New ids: append a random suffix so they don't clash with originals.
    const idMap = {};
    const stamp = Math.floor(Math.random() * 9000) + 1000;
    const newNodes = cb.nodes.map((n, i) => {
      const newId = `${n.id}_p${stamp}`;
      idMap[n.id] = newId;
      return {
        ...n,
        id: newId,
        position: { x: (n.position?.x ?? 0) + 40, y: (n.position?.y ?? 0) + 40 },
        selected: true,
        data: { ...n.data, label: newId, params: { ...n.data.params },
                onParamChange: handleParamChange },
      };
    });
    const newEdges = cb.edges
      .filter(e => idMap[e.source] && idMap[e.target])
      .map(e => ({
        ...e,
        id: `${idMap[e.source]}.${e.sourceHandle}->${idMap[e.target]}.${e.targetHandle}`,
        source: idMap[e.source],
        target: idMap[e.target],
        selected: true,
      }));
    setNodes(curr => [...curr.map(n => ({ ...n, selected: false })), ...newNodes]);
    setEdges(curr => [...curr.map(e => ({ ...e, selected: false })), ...newEdges]);
    setDirty(true);
  };

  const deleteSelection = () => {
    const selNids = new Set(nodes.filter(n => n.selected).map(n => n.id));
    const selEids = new Set(edges.filter(e => e.selected).map(e => e.id));
    if (selNids.size === 0 && selEids.size === 0) return;
    pushHistory();
    setNodes(curr => curr.filter(n => !selNids.has(n.id)));
    setEdges(curr => curr.filter(e =>
      !selEids.has(e.id) && !selNids.has(e.source) && !selNids.has(e.target)
    ));
    setDirty(true);
  };

  // Keyboard shortcuts: Ctrl/Cmd + Z / Shift+Z / C / V, Delete / Backspace.
  // Only active when the editor view is shown AND the focus isn't on an
  // <input>/<select>/<textarea> (so param editing isn't hijacked).
  useEffect(() => {
    const onKey = (e) => {
      if (typeof activeView !== 'undefined' && activeView !== 'vision-editor') return;
      const t = e.target;
      const tag = t && t.tagName ? t.tagName.toUpperCase() : '';
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' ||
          (t && t.isContentEditable)) return;
      const mod = e.ctrlKey || e.metaKey;
      if (mod && !e.shiftKey && (e.key === 'z' || e.key === 'Z')) {
        e.preventDefault(); undo();
      } else if (mod && (e.key === 'y' || e.key === 'Y' ||
                          (e.shiftKey && (e.key === 'z' || e.key === 'Z')))) {
        e.preventDefault(); redo();
      } else if (mod && (e.key === 'c' || e.key === 'C')) {
        e.preventDefault(); copySelection();
      } else if (mod && (e.key === 'v' || e.key === 'V')) {
        e.preventDefault(); pasteSelection();
      } else if (e.key === 'Delete' || e.key === 'Backspace') {
        // Don't intercept Backspace when typing in a non-editable element
        if (e.key === 'Backspace' && !mod) e.preventDefault();
        deleteSelection();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [nodes, edges]);

  // Initial load: fetch pipelines + node kinds, then auto-pause any video
  // source on the active pipeline + capture a snapshot for the freeze-frame
  // debug view.
  useEffect(() => {
    fetch('/api/vision/pipelines').then(r => r.json()).then(data => {
      setRegistry(data);
      if (data.pipelines && data.pipelines.length > 0 && !pipelineName) {
        loadPipeline(data.pipelines[0], data.node_kinds);
        setPipelineName(data.pipelines[0].name);
      }
      // Auto-pause + snap so switching from Dashboard (where a video may
      // be playing) to Editor freezes the playback and feeds the current
      // frame through the draft graph.
      setTimeout(() => captureSnapshot({ pauseFirst: true }), 200);
    });
  }, []);

  // The outer (vanilla) JS dispatches `vision-editor-activated` on the
  // window every time the user switches to the editor tab. We listen and
  // refresh the freeze-frame snap so the debug view always reflects the
  // current dashboard state.
  useEffect(() => {
    const onActivated = () => captureSnapshot({ pauseFirst: true });
    window.addEventListener('vision-editor-activated', onActivated);
    return () => window.removeEventListener('vision-editor-activated', onActivated);
  }, [registry, pipelineName]);

  // Periodic snap refresh — only useful for camera-style sources where the
  // scene changes over time. Driven by a 5 s timer that's gated on the
  // active pipeline containing a source.camera node.
  useEffect(() => {
    if (!registry) return;
    const active = (registry.pipelines || []).find(
      p => p.name === (registry.active || pipelineName)
    );
    const hasCamera = !!(active?.graph?.nodes || [])
      .find(n => n.kind === 'source.camera');
    if (!hasCamera) return;
    const t = setInterval(() => {
      // Don't yank a snap if the user is on a different tab.
      if (typeof activeView === 'undefined' ||
          activeView === 'vision-editor') {
        captureSnapshot();
      }
    }, 5000);
    return () => clearInterval(t);
  }, [registry, pipelineName]);

  // Live state poll (per-node states) every 1s
  useEffect(() => {
    if (!pipelineName) return;
    const t = setInterval(() => {
      fetch('/api/vision/pipelines').then(r => r.json()).then(data => {
        const live = data.pipelines.find(p => p.name === pipelineName);
        if (!live) return;
        setNodes(curr => curr.map(n => {
          const ns = (live.state.nodes || []).find(x => x.id === n.id);
          if (!ns) return n;
          return { ...n, data: { ...n.data, state: ns.state } };
        }));
      });
    }, 1000);
    return () => clearInterval(t);
  }, [pipelineName]);

  // SocketIO feed subscription
  useEffect(() => {
    if (typeof socket === 'undefined') return;
    const handler = (data) => {
      setFeeds(curr => ({
        ...curr,
        [data.feed_id]: { jpeg: data.jpeg, meta: data.meta, t: Date.now() },
      }));
    };
    socket.on('vision_feed', handler);
    return () => socket.off('vision_feed', handler);
  }, []);

  // Convert backend graph → React Flow nodes/edges
  const loadPipeline = (pipeline, nodeKinds) => {
    const kindMap = Object.fromEntries((nodeKinds || []).map(k => [k.kind, k]));
    const rfNodes = (pipeline.graph.nodes || []).map((nd, idx) => ({
      id:   nd.id,
      type: 'generic',
      // Restore the saved position if present; otherwise lay nodes out in
      // a default 3-column grid so a fresh graph isn't all stacked at 0,0.
      position: (nd.position && typeof nd.position.x === 'number')
        ? { x: nd.position.x, y: nd.position.y }
        : { x: 50 + (idx % 3) * 240, y: 50 + Math.floor(idx / 3) * 200 },
      data: {
        kind:   nd.kind,
        label:  nd.id,
        info:   kindMap[nd.kind] || { inputs: [], outputs: [], params_schema: {} },
        params: nd.params || {},
        state:  pipeline.state?.nodes?.find(x => x.id === nd.id)?.state || {},
        onParamChange: handleParamChange,
      },
    }));
    const rfEdges = [];
    (pipeline.graph.nodes || []).forEach(nd => {
      (nd.inputs || []).forEach(inp => {
        rfEdges.push({
          id: `${inp.src_node}.${inp.src_port}->${nd.id}.${inp.port}`,
          source: inp.src_node,
          sourceHandle: inp.src_port,
          target: nd.id,
          targetHandle: inp.port,
          style: { stroke: '#94a3b8', strokeWidth: 1.5 },
          interactionWidth: 18,
        });
      });
    });
    // Loading a graph isn't an edit — skip history and reset stacks.
    skipHistoryRef.current = true;
    setNodes(rfNodes);
    setEdges(rfEdges);
    setTimeout(() => { skipHistoryRef.current = false; }, 0);
    historyRef.current = { past: [], future: [] };
    setDirty(false);
  };

  const handleParamChange = (nodeId, key, val) => {
    pushHistory();
    setNodes(curr => curr.map(n => {
      if (n.id !== nodeId) return n;
      return { ...n, data: { ...n.data, params: { ...n.data.params, [key]: val } } };
    }));
    setDirty(true);
  };

  // Look up a port kind given a node id + handle id.
  const _portKind = (nodeId, handleId, side /* 'inputs' | 'outputs' */) => {
    const n = nodes.find(x => x.id === nodeId);
    if (!n) return null;
    const ports = n.data?.info?.[side] || [];
    const p = ports.find(x => x.name === handleId);
    return p ? p.kind : null;
  };

  const handleConnect = useCallback(params => {
    // Refuse mismatched port kinds — without this, you can wire a
    // pose_list into a frame input, the pipeline silently runs
    // garbage in, and the user spends an hour staring at a black tile.
    // FRAME → PREVIEW is allowed (clean frame is a valid display backdrop)
    // PREVIEW → FRAME is rejected (don't process annotated images).
    const srcKind = _portKind(params.source, params.sourceHandle, 'outputs');
    const dstKind = _portKind(params.target, params.targetHandle, 'inputs');
    if (!isConnectionValid(srcKind, dstKind)) {
      // Surface the rejection visibly so the user knows why the wire
      // didn't take. (In-page toast if available, console fallback.)
      const msg = `Can't connect ${srcKind} → ${dstKind}`;
      if (typeof showToast === 'function') showToast(msg);
      console.warn('[vision-editor]', msg, params);
      return;
    }
    pushHistory();
    setEdges(eds => addEdge({
      ...params,
      style: { stroke: KIND_COLOR[srcKind] || '#94a3b8', strokeWidth: 1.5 },
      interactionWidth: 18,
    }, eds));
    setDirty(true);
  }, [nodes, edges]);

  const addNodeOfKind = (kind) => {
    if (!registry) return;
    const info = registry.node_kinds.find(k => k.kind === kind);
    if (!info) return;
    pushHistory();
    const id = `${kind.split('.').pop()}_${Math.floor(Math.random() * 9000) + 1000}`;
    const defaults = Object.fromEntries(
      Object.entries(info.params_schema || {}).map(([k, d]) => [k, d.default])
    );
    setNodes(curr => [...curr, {
      id, type: 'generic',
      position: { x: 80 + Math.random() * 300, y: 80 + Math.random() * 200 },
      data: { kind, label: id, info, params: defaults, state: {},
              onParamChange: handleParamChange },
    }]);
    setDirty(true);
  };

  // Convert React Flow graph → backend dict. Saving does NOT activate the
  // pipeline — that's a deliberate decision so the user can edit a graph
  // without disturbing the currently-running one. Use the dashboard picker
  // to swap which pipeline drives the feeds.
  const exportGraph = () => {
    const inputsByDst = {};
    edges.forEach(e => {
      if (!inputsByDst[e.target]) inputsByDst[e.target] = [];
      inputsByDst[e.target].push({
        port: e.targetHandle, src_node: e.source, src_port: e.sourceHandle,
      });
    });
    return {
      name: pipelineName,
      fps_limit: 25,
      nodes: nodes.map(n => ({
        id:     n.id,
        kind:   n.data.kind,
        params: n.data.params,
        inputs: inputsByDst[n.id] || [],
        // Persist the React Flow position so layout survives a save+
        // reload. Without this, every reload re-laid the graph in a
        // 3-column grid regardless of where the user dragged things.
        position: n.position
          ? { x: Math.round(n.position.x), y: Math.round(n.position.y) }
          : null,
      })),
    };
  };

  const savePipeline = async () => {
    if (!pipelineName) return;
    const graph = exportGraph();
    const r = await fetch(`/api/vision/pipelines/${encodeURIComponent(pipelineName)}`, {
      method: 'PUT', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(graph),
    });
    const data = await r.json();
    if (data.ok) {
      setDirty(false);
      // Refresh the registry payload so the dashboard picker sees the
      // new graph. Active pipeline is preserved (or cleared if we just
      // saved over the active one — that's intentional).
      const fresh = await (await fetch('/api/vision/pipelines')).json();
      setRegistry(fresh);
    } else {
      alert(`Save failed: ${data.error || 'unknown'}`);
    }
  };

  const setAsActive = async () => {
    if (!pipelineName) return;
    await fetch('/api/vision/active', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name: pipelineName }),
    });
    const fresh = await (await fetch('/api/vision/pipelines')).json();
    setRegistry(fresh);
  };

  const newPipeline = () => {
    const name = prompt('Pipeline name:');
    if (!name) return;
    setPipelineName(name);
    setNodes([]); setEdges([]); setDirty(true);
  };

  const deletePipeline = async () => {
    if (!pipelineName) return;
    if (!confirm(`Delete pipeline "${pipelineName}"?`)) return;
    await fetch(`/api/vision/pipelines/${encodeURIComponent(pipelineName)}`,
                { method: 'DELETE' });
    setPipelineName(null);
    setNodes([]); setEdges([]);
    fetch('/api/vision/pipelines').then(r => r.json()).then(setRegistry);
  };

  // Editor available?
  if (!registry) {
    return <div style={{ padding: 12, color: '#888' }}>Loading registry…</div>;
  }
  if (!registry.available) {
    return <div style={{ padding: 12, color: '#c33' }}>
      Pipeline registry unavailable (OpenCV missing?).
    </div>;
  }

  return (
    <div className="vne-root">
      {/* Top bar */}
      <div className="vne-toolbar">
        <select className="input-sm" value={pipelineName || ''}
                onChange={e => {
                  const name = e.target.value;
                  const p = registry.pipelines.find(x => x.name === name);
                  if (p) { setPipelineName(name); loadPipeline(p, registry.node_kinds); }
                }}>
          <option value="">— pick pipeline —</option>
          {registry.pipelines.map(p =>
            <option key={p.name} value={p.name}>
              {p.name}{p.enabled ? ' ●' : ''}
            </option>)}
        </select>
        <button className="btn-tiny" onClick={newPipeline}>+ new</button>
        <button className="btn-tiny" onClick={undo}
                disabled={historyRef.current.past.length === 0}
                title="Undo (Ctrl+Z)">↶ undo</button>
        <button className="btn-tiny" onClick={redo}
                disabled={historyRef.current.future.length === 0}
                title="Redo (Ctrl+Shift+Z)">↷ redo</button>
        <button className="btn-tiny" onClick={copySelection}
                title="Copy selected (Ctrl+C)">⎘ copy</button>
        <button className="btn-tiny" onClick={pasteSelection}
                title="Paste (Ctrl+V)">paste</button>
        <button className="btn-tiny" onClick={deleteSelection}
                title="Delete selected (Del)">✕ del</button>
        <span style={{ width: 1, height: 16, background: 'var(--surface2)', margin: '0 4px' }}></span>
        <button className="btn-tiny" onClick={captureSnapshot}
                title="Capture one frame from the active pipeline's source for debug">
          📷 snap
        </button>
        <button className="btn-tiny" onClick={runDebug}
                disabled={!snapshot}
                title="Run the editor's draft graph against the snapshot frame">
          ▶ exec
        </button>
        <span style={{ width: 1, height: 16, background: 'var(--surface2)', margin: '0 4px' }}></span>
        <button className="btn-tiny" onClick={savePipeline}
                disabled={!dirty || !pipelineName}>
          {dirty ? 'save *' : 'save'}
        </button>
        <button className="btn-tiny" onClick={setAsActive}
                disabled={!pipelineName || registry?.active === pipelineName}
                title="Make this pipeline drive the dashboard feeds">
          {registry?.active === pipelineName ? '● active' : '▶ set active'}
        </button>
        <button className="btn-tiny" onClick={deletePipeline}
                disabled={!pipelineName}>delete</button>
        <span style={{ flex: 1 }} />
        <span className="vne-snap-status">
          {snapshot
            ? `📸 ${snapshotInfo?.shape ? snapshotInfo.shape.slice(0,2).reverse().join('×') : 'snapped'}`
            : (snapshotInfo?.error ? `⚠ ${snapshotInfo.error}` : 'no snapshot — click 📷 snap')}
        </span>
        <details className="vne-add-menu">
          <summary>+ add node</summary>
          <div>
            {registry.node_kinds.map(k =>
              <button key={k.kind} className="btn-tiny" onClick={() => addNodeOfKind(k.kind)}>
                {k.kind}
              </button>)}
          </div>
        </details>
      </div>

      {/* Canvas + dashboard */}
      <div className="vne-content">
        <div className="vne-canvas">
          <ReactFlowProvider>
            <ReactFlow
              nodes={nodes}
              edges={edges}
              onNodesChange={onNodesChange}
              onEdgesChange={(c) => { onEdgesChange(c); setDirty(true); }}
              onConnect={handleConnect}
              nodeTypes={NODE_TYPES}
              fitView
            >
              <Background color="#444" gap={20} />
              <Controls />
            </ReactFlow>
          </ReactFlowProvider>
        </div>

        {/* Dashboard (right) */}
        <div className="vne-dashboard">
          <div className="vne-dashboard-header">Live feeds</div>
          {Object.keys(feeds).length === 0 ? (
            <div style={{ color: '#888', padding: 12, fontSize: 11 }}>
              No feeds yet. Add an <code>output</code> node to a pipeline,
              set its <code>feed_id</code>, then save+enable.
            </div>
          ) : Object.entries(feeds).map(([fid, f]) => (
            <div key={fid} className="vne-feed-card">
              <div className="vne-feed-label">{f.meta?.label || fid}</div>
              <img src={`data:image/jpeg;base64,${f.jpeg}`} alt={fid} />
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

// Mount when the visio tab is shown — but only the FIRST time.
let _editorMounted = false;
function visionMountEditor() {
  if (_editorMounted) return;
  const host = document.getElementById('vision-editor-root');
  if (!host) return;
  if (typeof React === 'undefined' || typeof ReactDOM === 'undefined' || !window.ReactFlow) {
    host.innerHTML = '<div style="padding:12px;color:#c33">' +
      'Editor libs not loaded (React / ReactFlow CDN failed). ' +
      'Check network access to cdnjs.cloudflare.com / unpkg.com.</div>';
    return;
  }
  ReactDOM.createRoot(host).render(<PipelineEditor />);
  _editorMounted = true;
}

window.visionMountEditor = visionMountEditor;
