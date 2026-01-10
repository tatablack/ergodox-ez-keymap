const key_override_t delete_key_override = ko_make_basic(
    MOD_MASK_SHIFT, // trigger_mods: Shift
    KC_BSPC,        // trigger key: Backspace
    KC_DEL          // replacement: Delete
);

const key_override_t *key_overrides[] = {
    &delete_key_override,
 };

