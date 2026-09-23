# World previews

This directory owns presentation images for public simulator world identities.
`catalog.json` is deliberately small and UI-neutral: a future world picker can
filter by `backend`, show `image`, and launch with `public_world` without learning
the dependency's internal filenames.

The Gazebo images are copied unchanged from the documentation assets in the
pinned Clearpath simulator checkout. Add a catalog entry only when a useful
world render exists; occupancy maps belong in `../ground_truth_maps/` instead.
