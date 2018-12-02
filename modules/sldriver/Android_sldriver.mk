ifeq ($(INTEL_FEATURE_SILENTLAKE),true)
$(eval $(call build_kernel_module,$(call my-dir),vidt_driver))
endif
