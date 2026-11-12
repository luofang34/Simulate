//! The worker that writes an [`super::OpfsStore`]'s files through
//! synchronous access handles. A main-thread writable stream leaves a swap
//! file in the browser's temp folder for every file in Safari (500 left by
//! 500 writes, measured on Safari 27); a sync access handle, which only a
//! worker may open, writes in place and leaves none, in about half the
//! time.
//!
//! The worker is the page's `opfs-writer.js`. It answers each
//! `{ id, root, path, bytes }` with `{ id, ok }`, or with `unsupported` when
//! the browser has no sync access handles.

use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::rc::Rc;

use js_sys::{Array, Function, Object, Promise, Reflect, Uint8Array};
use navigate_data::DataError;
use wasm_bindgen::closure::Closure;
use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use web_sys::{Event, MessageEvent, Worker};

use super::super::js_text;

/// The settle functions of each write awaiting the worker's answer.
type Pending = Rc<RefCell<BTreeMap<u32, (Function, Function)>>>;

/// What became of a write handed to the worker.
pub(super) enum Written {
    /// The file holds the bytes.
    Done,
    /// The worker cannot write here: no sync access handles, or the
    /// worker did not start. The reason, for the log.
    Unavailable(String),
}

/// A running writer worker.
pub(super) struct OpfsWriter {
    worker: Worker,
    root: String,
    next_id: Cell<u32>,
    pending: Pending,
    _on_message: Closure<dyn FnMut(MessageEvent)>,
    _on_error: Closure<dyn FnMut(Event)>,
}

impl Drop for OpfsWriter {
    fn drop(&mut self) {
        self.worker.terminate();
    }
}

impl OpfsWriter {
    /// Starts the worker at `url`, writing under the directory `root` of
    /// the origin private file system.
    pub(super) fn start(url: &str, root: &str) -> Result<Self, String> {
        let worker = Worker::new(url).map_err(js_text)?;
        let pending = Pending::default();
        let answered = Rc::clone(&pending);
        let on_message: Closure<dyn FnMut(MessageEvent)> =
            Closure::new(move |event: MessageEvent| {
                let reply = event.data();
                let id = Reflect::get(&reply, &"id".into())
                    .ok()
                    .and_then(|id| id.as_f64())
                    .and_then(|id| u32::try_from(id as u64).ok());
                if let Some((resolve, _)) = id.and_then(|id| answered.borrow_mut().remove(&id)) {
                    resolve.call1(&JsValue::NULL, &reply).ok();
                }
            });
        // A worker that fails to load or throws answers nothing: every write
        // waiting on it is refused, so the store writes on the main thread.
        let failed = Rc::clone(&pending);
        let on_error: Closure<dyn FnMut(Event)> = Closure::new(move |_event: Event| {
            let reason = JsValue::from_str("the writer worker failed");
            for (_, (_, reject)) in std::mem::take(&mut *failed.borrow_mut()) {
                reject.call1(&JsValue::NULL, &reason).ok();
            }
        });
        worker.set_onmessage(Some(on_message.as_ref().unchecked_ref()));
        worker.set_onerror(Some(on_error.as_ref().unchecked_ref()));
        Ok(Self {
            worker,
            root: root.to_owned(),
            next_id: Cell::new(0),
            pending,
            _on_message: on_message,
            _on_error: on_error,
        })
    }

    /// Writes `bytes` as the whole file at `path`.
    pub(super) async fn write(&self, path: &str, bytes: &[u8]) -> Result<Written, DataError> {
        let id = self.next_id.get();
        self.next_id.set(id.wrapping_add(1));
        let buffer = Uint8Array::from(bytes).buffer();
        let message = Object::new();
        for (key, value) in [
            ("id", JsValue::from(id)),
            ("root", JsValue::from_str(&self.root)),
            ("path", JsValue::from_str(path)),
            ("bytes", buffer.clone().into()),
        ] {
            Reflect::set(&message, &key.into(), &value)
                .map_err(|error| store_error(path, error))?;
        }
        let pending = Rc::clone(&self.pending);
        let answer = Promise::new(&mut |resolve, reject| {
            pending.borrow_mut().insert(id, (resolve, reject));
        });
        if let Err(error) = self
            .worker
            .post_message_with_transfer(&message, &Array::of1(&buffer))
        {
            self.pending.borrow_mut().remove(&id);
            return Ok(Written::Unavailable(js_text(error)));
        }
        let reply = match JsFuture::from(answer).await {
            Ok(reply) => reply,
            Err(error) => return Ok(Written::Unavailable(js_text(error))),
        };
        let field = |name: &str| Reflect::get(&reply, &name.into()).unwrap_or(JsValue::UNDEFINED);
        if field("ok").as_bool() == Some(true) {
            return Ok(Written::Done);
        }
        let reason = field("error")
            .as_string()
            .unwrap_or_else(|| "the writer worker gave no reason".to_owned());
        if field("unsupported").as_bool() == Some(true) {
            return Ok(Written::Unavailable(reason));
        }
        Err(DataError::Store {
            path: path.to_owned(),
            message: reason,
        })
    }
}

fn store_error(path: &str, error: JsValue) -> DataError {
    DataError::Store {
        path: path.to_owned(),
        message: js_text(error),
    }
}
