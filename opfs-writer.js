// Writes files of the page's origin private file system for sim-navdata's
// OpfsStore, through synchronous access handles. A main-thread writable
// stream leaves a swap file in the browser's temp folder for every file in
// Safari; a sync access handle writes in place and leaves none.
//
// Message in:  { id, root, path, bytes }   (bytes: ArrayBuffer)
// Message out: { id, ok: true } or { id, ok: false, error, unsupported }
// `unsupported` says the browser has no sync access handles, so the store
// writes on the main thread instead.

async function directoryOf(root, path) {
  let directory = await (await navigator.storage.getDirectory()).getDirectoryHandle(root, {
    create: true,
  });
  const parts = path.split('/');
  const name = parts.pop();
  for (const part of parts) {
    directory = await directory.getDirectoryHandle(part, { create: true });
  }
  return [directory, name];
}

onmessage = async (event) => {
  const { id, root, path, bytes } = event.data;
  try {
    const [directory, name] = await directoryOf(root, path);
    const handle = await directory.getFileHandle(name, { create: true });
    if (typeof handle.createSyncAccessHandle !== 'function') {
      postMessage({ id, ok: false, unsupported: true, error: 'no sync access handles' });
      return;
    }
    const access = await handle.createSyncAccessHandle();
    try {
      access.truncate(0);
      const view = new Uint8Array(bytes);
      let at = 0;
      while (at < view.length) {
        const written = access.write(view.subarray(at), { at });
        if (written <= 0) throw new Error(`wrote ${at} of ${view.length} bytes`);
        at += written;
      }
      access.flush();
    } finally {
      access.close();
    }
    postMessage({ id, ok: true });
  } catch (error) {
    postMessage({ id, ok: false, error: `${error && error.name}: ${error && error.message}` });
  }
};
