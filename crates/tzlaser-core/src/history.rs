use crate::Document;

/// Snapshot of the document and active selection at a point in time.
#[derive(Clone, Debug)]
pub struct Snapshot {
    pub document: Document,
    pub selection: Vec<(usize, usize)>,
    pub description: String,
}

/// Undo/Redo history stack manager.
#[derive(Clone, Debug)]
pub struct HistoryManager {
    undo_stack: Vec<Snapshot>,
    redo_stack: Vec<Snapshot>,
    max_history: usize,
}

impl Default for HistoryManager {
    fn default() -> Self {
        Self::new(60)
    }
}

impl HistoryManager {
    pub fn new(max_history: usize) -> Self {
        Self {
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            max_history,
        }
    }

    /// Record a snapshot before applying a mutating operation.
    /// Clears redo history.
    pub fn record(&mut self, document: &Document, selection: &[(usize, usize)], description: impl Into<String>) {
        if self.undo_stack.len() >= self.max_history {
            self.undo_stack.remove(0);
        }
        self.undo_stack.push(Snapshot {
            document: document.clone(),
            selection: selection.to_vec(),
            description: description.into(),
        });
        self.redo_stack.clear();
    }

    /// Undo the last action, restoring document and selection.
    /// Pushes current state to the redo stack.
    pub fn undo(&mut self, document: &mut Document, selection: &mut Vec<(usize, usize)>) -> Option<String> {
        let prev = self.undo_stack.pop()?;
        self.redo_stack.push(Snapshot {
            document: document.clone(),
            selection: selection.clone(),
            description: prev.description.clone(),
        });
        *document = prev.document;
        *selection = prev.selection;
        Some(prev.description)
    }

    /// Redo the last undone action.
    /// Pushes current state back to the undo stack.
    pub fn redo(&mut self, document: &mut Document, selection: &mut Vec<(usize, usize)>) -> Option<String> {
        let next = self.redo_stack.pop()?;
        self.undo_stack.push(Snapshot {
            document: document.clone(),
            selection: selection.clone(),
            description: next.description.clone(),
        });
        *document = next.document;
        *selection = next.selection;
        Some(next.description)
    }

    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    pub fn undo_description(&self) -> Option<&str> {
        self.undo_stack.last().map(|s| s.description.as_str())
    }

    pub fn redo_description(&self) -> Option<&str> {
        self.redo_stack.last().map(|s| s.description.as_str())
    }

    pub fn clear(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Color, Layer, Entity};
    use kurbo::Point;

    #[test]
    fn test_undo_redo_cycle() {
        let mut doc = Document::default();
        let mut selection = Vec::new();
        let mut history = HistoryManager::default();

        assert!(!history.can_undo());
        assert!(!history.can_redo());

        // Initial state: 0 entities
        // Action 1: Add circle
        history.record(&doc, &selection, "Add circle");
        let mut layer = Layer::new("Cut", Color::RED);
        layer.entities.push(Entity::Circle { center: Point::new(10.0, 10.0), radius: 5.0 });
        doc.layers = vec![layer];
        selection = vec![(0, 0)];

        assert_eq!(doc.entity_count(), 1);
        assert!(history.can_undo());
        assert!(!history.can_redo());

        // Action 2: Translate circle
        history.record(&doc, &selection, "Translate circle");
        doc.translate_selection(&selection, 20.0, 30.0);

        if let Entity::Circle { center, .. } = &doc.layers[0].entities[0] {
            assert_eq!(center.x, 30.0);
            assert_eq!(center.y, 40.0);
        }

        // Undo Action 2: Should restore circle to (10, 10)
        let desc = history.undo(&mut doc, &mut selection);
        assert_eq!(desc.as_deref(), Some("Translate circle"));
        assert!(history.can_redo());
        if let Entity::Circle { center, .. } = &doc.layers[0].entities[0] {
            assert_eq!(center.x, 10.0);
            assert_eq!(center.y, 10.0);
        }

        // Redo Action 2: Should re-translate circle to (30, 40)
        let redo_desc = history.redo(&mut doc, &mut selection);
        assert_eq!(redo_desc.as_deref(), Some("Translate circle"));
        if let Entity::Circle { center, .. } = &doc.layers[0].entities[0] {
            assert_eq!(center.x, 30.0);
            assert_eq!(center.y, 40.0);
        }

        // Undo twice back to initial state
        history.undo(&mut doc, &mut selection);
        history.undo(&mut doc, &mut selection);
        assert_eq!(doc.entity_count(), 0);
        assert!(!history.can_undo());
        assert!(history.can_redo());
    }
}
