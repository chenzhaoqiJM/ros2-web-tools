(() => {
  const $ = id => document.getElementById(id);
  const state = {
    topics: [], selected: null, filter: 'all', pc: null, peerId: null,
    profile: 'balanced', stats: null, stable: 0, connecting: false,
  };
  const labels = {rgb: 'RGB', depth: '深度', infrared: '红外'};
  const qualityLabels = {low: '流畅', balanced: '均衡', high: '高清'};

  function escapeHtml(value) {
    const node = document.createElement('span');
    node.textContent = value;
    return node.innerHTML;
  }

  function formatAge(age) {
    if (age == null) return '等待';
    if (age < 1) return '实时';
    if (age < 10) return `${age.toFixed(1)} 秒前`;
    return '已暂停';
  }

  function renderSources() {
    const visible = state.topics.filter(item => state.filter === 'all' || item.kind === state.filter);
    $('source-count').textContent = state.topics.length;
    if (!visible.length) {
      $('source-list').innerHTML = `<div class="empty-source"><span class="scan-icon"></span><p>${state.topics.length ? '该类型暂无图像话题' : '正在扫描 ROS 2 图像话题…'}</p></div>`;
      return;
    }
    $('source-list').innerHTML = visible.map(item => {
      const active = item.topic === state.selected ? ' active' : '';
      const live = item.age != null && item.age < 2;
      const size = item.width ? `${item.width} × ${item.height}` : '等待首帧';
      return `<button class="source${active}" data-topic="${escapeHtml(item.topic)}">
        <span class="source-icon ${item.kind}"><i></i></span>
        <span class="source-copy"><code>${escapeHtml(item.topic)}</code><small>${labels[item.kind] || '图像'} · ${size}</small></span>
        <span class="source-state ${live ? 'live' : ''}">${formatAge(item.age)}</span>
      </button>`;
    }).join('');
    document.querySelectorAll('.source').forEach(button => {
      button.addEventListener('click', () => selectTopic(button.dataset.topic));
    });
  }

  function updateDetails(item) {
    const live = item && item.age != null && item.age < 3;
    $('connection').className = `connection ${live ? 'live' : item ? 'warn' : ''}`;
    $('connection').lastElementChild.textContent = live ? '低延迟连接' : item ? '等待图像数据' : '未发现图像话题';
    $('kind-badge').textContent = item ? labels[item.kind] : '未选择';
    $('kind-badge').className = `kind-badge ${item ? item.kind : ''}`;
    $('active-topic').textContent = item?.topic || '—';
    $('resolution').textContent = item?.width ? `${item.width} × ${item.height}` : '—';
    $('message-type').textContent = item?.message_type?.split('/').pop() || '—';
    $('encoding').textContent = item?.encoding || '—';
    $('frame-count').textContent = item ? item.frame_count.toLocaleString() : '—';
    $('value-range').textContent = item?.range ? `${formatRange(item.range[0], item.encoding)} – ${formatRange(item.range[1], item.encoding)}` : '—';
  }

  function formatRange(value, encoding) {
    if ((encoding || '').toLowerCase().includes('32fc1')) return `${value.toFixed(2)} m`;
    if ((encoding || '').toLowerCase().includes('16')) return `${Math.round(value)} mm`;
    return value.toFixed(1);
  }

  async function refreshTopics() {
    try {
      const response = await fetch('/api/topics', {cache: 'no-store'});
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      state.topics = (await response.json()).topics;
      if (state.selected && !state.topics.some(item => item.topic === state.selected)) {
        await stopStream();
        state.selected = null;
      }
      if (!state.selected && state.topics.length) {
        const ready = state.topics.find(item => item.frame_count > 0) || state.topics[0];
        selectTopic(ready.topic);
      }
      renderSources();
      updateDetails(state.topics.find(item => item.topic === state.selected));
    } catch (error) {
      $('connection').className = 'connection error';
      $('connection').lastElementChild.textContent = '服务连接中断';
    }
  }

  function waitIceComplete(pc) {
    if (pc.iceGatheringState === 'complete') return Promise.resolve();
    return new Promise(resolve => {
      const check = () => {
        if (pc.iceGatheringState === 'complete') {
          pc.removeEventListener('icegatheringstatechange', check);
          resolve();
        }
      };
      pc.addEventListener('icegatheringstatechange', check);
    });
  }

  async function selectTopic(topic) {
    if (state.selected === topic && state.pc) return;
    state.selected = topic;
    state.stats = null;
    state.stable = 0;
    renderSources();
    updateDetails(state.topics.find(item => item.topic === topic));
    await startStream(topic);
  }

  async function startStream(topic) {
    if (state.connecting) return;
    state.connecting = true;
    $('no-video').classList.remove('hidden');
    $('no-video').querySelector('h2').textContent = '正在建立 WebRTC 连接';
    $('no-video').querySelector('p').textContent = topic;
    await stopStream();
    const pc = new RTCPeerConnection({bundlePolicy: 'max-bundle'});
    state.pc = pc;
    pc.addTransceiver('video', {direction: 'recvonly'});
    pc.ontrack = event => {
      $('video').srcObject = event.streams[0] || new MediaStream([event.track]);
      $('no-video').classList.add('hidden');
      $('snapshot').disabled = false;
    };
    pc.onconnectionstatechange = () => {
      if (state.pc !== pc) return;
      if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
        $('connection').className = 'connection error';
        $('connection').lastElementChild.textContent = 'WebRTC 连接中断';
      }
    };
    try {
      await pc.setLocalDescription(await pc.createOffer());
      await waitIceComplete(pc);
      const response = await fetch('/api/offer', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({sdp: pc.localDescription.sdp, type: pc.localDescription.type,
                              topic, quality: state.profile}),
      });
      if (!response.ok) throw new Error(await response.text());
      const answer = await response.json();
      state.peerId = answer.peer_id;
      await pc.setRemoteDescription(answer);
    } catch (error) {
      console.error(error);
      if (state.pc === pc) {
        $('no-video').classList.remove('hidden');
        $('no-video').querySelector('h2').textContent = '视频连接失败';
        $('no-video').querySelector('p').textContent = '请检查服务端依赖与网络设置';
      }
      pc.close();
    } finally {
      state.connecting = false;
    }
  }

  async function stopStream() {
    const peerId = state.peerId;
    state.peerId = null;
    const pc = state.pc;
    state.pc = null;
    if (pc) pc.close();
    if (peerId) fetch(`/api/peers/${peerId}`, {method: 'DELETE', keepalive: true}).catch(() => {});
    $('video').srcObject = null;
    $('snapshot').disabled = true;
  }

  async function setProfile(profile) {
    if (state.profile === profile) return;
    state.profile = profile;
    $('active-quality').textContent = qualityLabels[profile];
    if (state.peerId) {
      fetch(`/api/peers/${state.peerId}/quality`, {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({quality: profile}),
      }).catch(console.error);
    }
  }

  async function updateStats() {
    const pc = state.pc;
    if (!pc || pc.connectionState !== 'connected') return;
    const reports = await pc.getStats();
    let inbound, candidate;
    reports.forEach(report => {
      if (report.type === 'inbound-rtp' && (report.kind === 'video' || report.mediaType === 'video')) inbound = report;
      if (report.type === 'candidate-pair' && report.state === 'succeeded' && report.nominated) candidate = report;
    });
    if (!inbound) return;
    const now = inbound.timestamp;
    let bitrate = null, fps = inbound.framesPerSecond || null, loss = 0;
    if (state.stats) {
      const seconds = (now - state.stats.time) / 1000;
      bitrate = seconds > 0 ? (inbound.bytesReceived - state.stats.bytes) * 8 / seconds : null;
      const lost = (inbound.packetsLost || 0) - state.stats.lost;
      const received = (inbound.packetsReceived || 0) - state.stats.received;
      loss = Math.max(0, lost / Math.max(1, lost + received));
      if (!fps && seconds > 0) fps = (inbound.framesDecoded - state.stats.frames) / seconds;
    }
    state.stats = {time: now, bytes: inbound.bytesReceived || 0, lost: inbound.packetsLost || 0,
                   received: inbound.packetsReceived || 0, frames: inbound.framesDecoded || 0};
    $('fps').textContent = fps != null ? `${fps.toFixed(1)} fps` : '—';
    $('bitrate').textContent = bitrate != null ? `${(bitrate / 1e6).toFixed(2)} Mbps` : '—';
    $('latency').textContent = candidate?.currentRoundTripTime != null ? `${Math.round(candidate.currentRoundTripTime * 1000)} ms` : '—';
    $('loss').textContent = `${(loss * 100).toFixed(1)}%`;
    if ($('quality').value === 'auto' && bitrate != null) adaptQuality(loss, bitrate);
  }

  function adaptQuality(loss, bitrate) {
    const levels = ['low', 'balanced', 'high'];
    let index = levels.indexOf(state.profile);
    const tooSlow = loss > .05 || (state.profile === 'high' && bitrate < 900000) ||
      (state.profile === 'balanced' && bitrate < 350000);
    if (tooSlow && index > 0) {
      setProfile(levels[index - 1]);
      state.stable = 0;
      $('adapt-state').textContent = '网络波动 · 已降档';
      return;
    }
    state.stable = loss < .01 ? state.stable + 1 : 0;
    if (state.stable >= 5 && index < 2) {
      setProfile(levels[index + 1]);
      state.stable = 0;
      $('adapt-state').textContent = '网络良好 · 已升档';
    } else {
      $('adapt-state').textContent = loss < .02 ? '网络稳定' : '自动调节';
    }
  }

  document.querySelectorAll('[data-kind]').forEach(button => {
    button.addEventListener('click', () => {
      state.filter = button.dataset.kind;
      document.querySelectorAll('[data-kind]').forEach(item => item.classList.toggle('active', item === button));
      renderSources();
    });
  });
  $('quality').addEventListener('change', event => {
    const value = event.target.value;
    $('adapt-state').textContent = value === 'auto' ? '自动调节' : '手动设置';
    setProfile(value === 'auto' ? 'balanced' : value);
  });
  $('snapshot').addEventListener('click', () => {
    const video = $('video'), canvas = $('snapshot-canvas');
    canvas.width = video.videoWidth; canvas.height = video.videoHeight;
    canvas.getContext('2d').drawImage(video, 0, 0);
    const link = document.createElement('a');
    link.download = `ros-camera-${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
    link.href = canvas.toDataURL('image/jpeg', .95); link.click();
    $('toast').textContent = '已保存当前帧'; $('toast').classList.add('show');
    setTimeout(() => $('toast').classList.remove('show'), 1800);
  });
  $('fullscreen').addEventListener('click', () => {
    if (document.fullscreenElement) document.exitFullscreen(); else $('viewer').requestFullscreen();
  });
  window.addEventListener('beforeunload', stopStream);
  refreshTopics();
  setInterval(refreshTopics, 2000);
  setInterval(() => updateStats().catch(console.error), 2000);
})();
