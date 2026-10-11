/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Included after the retained DART and allocation helpers in apple-dart.c. */

static int apple_dart_dcp_shared_pair(struct device *dev, struct device *piodma,
				      struct apple_dart_dcp **firmware,
				      struct apple_dart_dcp **frontend)
{
	struct apple_dart_domain *a, *b;
	int ret;

	if (!dev || !piodma || dev == piodma || piodma->parent != dev ||
	    dev_is_dma_coherent(dev) || dev_is_dma_coherent(piodma))
		return -ENODEV;
	ret = apple_dart_dcp_check(dev);
	if (ret)
		return ret;
	ret = apple_dart_dcp_check(piodma);
	if (ret)
		return ret;
	a = to_dart_domain(iommu_get_domain_for_dev(dev));
	b = to_dart_domain(iommu_get_domain_for_dev(piodma));
	if (a->dcp->sid != 23 || b->dcp->sid != 16 || a->dcp->dart == b->dcp->dart)
		return -ENODEV;
	*firmware = a->dcp;
	*frontend = b->dcp;
	return 0;
}

static int apple_dart_dcp_shared_verify(struct apple_dart_dcp *dcp,
					struct apple_dart_dcp_shared *object)
{
	for (size_t offset = 0; offset < object->size; offset += PAGE_SIZE) {
		u64 dva = object->dva + offset;
		unsigned int root = (dva >> 25) & 2047, leaf = (dva >> 14) & 2047;

		if (apple_dart_dcp_translate(dcp, dva) != object->phys + offset ||
		    test_bit(root, dcp->foreign) || !test_bit(root, dcp->claimed) ||
		    !dcp_dart_pte_frontend(READ_ONCE(dcp->table[root][leaf])))
			return -EIO;
	}
	return 0;
}

int apple_dart_dcp_shared_alloc(struct device *dev, struct device *piodma,
				size_t size, u64 *phys, u64 *dva)
{
	struct apple_dart_dcp *a, *b;
	struct apple_dart_dcp_shared *object;
	unsigned int root;
	unsigned long flags;
	size_t mapped = 0;
	int ret;

	if (!phys || !dva || !size || size > SZ_16M)
		return -EINVAL;
	ret = apple_dart_dcp_shared_pair(dev, piodma, &a, &b);
	if (ret)
		return ret;
	size = PAGE_ALIGN(size);
	/* Always SID23 then SID16; ordinary RTKit allocation holds SID23 only. */
	mutex_lock(&a->alloc_lock);
	mutex_lock_nested(&b->alloc_lock, SINGLE_DEPTH_NESTING);
	ret = -EIO;
	if (READ_ONCE(a->failed) || READ_ONCE(b->failed))
		goto unlock;
	ret = -ENOSPC;
	if (a->shared_count >= 64 || a->allocated > SZ_64M - size || b->allocated > SZ_64M - size)
		goto unlock;
	for (root = 0; root < DCP_DART_ENTRIES; root++)
		if (!test_bit(root, a->claimed) && !test_bit(root, b->claimed) &&
		    !a->entry[root] && !b->entry[root] &&
		    !READ_ONCE(a->root[root]) && !READ_ONCE(b->root[root]))
			break;
	if (root == DCP_DART_ENTRIES)
		goto unlock;
	/* Serialize the claim against explicit generic map/unmap callers too. */
	spin_lock_irqsave(&a->lock, flags);
	spin_lock_nested(&b->lock, SINGLE_DEPTH_NESTING);
	if (a->entry[root] || b->entry[root] || READ_ONCE(a->root[root]) || READ_ONCE(b->root[root])) {
		spin_unlock(&b->lock);
		spin_unlock_irqrestore(&a->lock, flags);
		ret = -EBUSY;
		goto unlock;
	}
	/* Failed attempts retain both claims, even before any page is published. */
	set_bit(root, a->claimed);
	set_bit(root, b->claimed);
	set_bit(root, a->shared_roots);
	set_bit(root, b->shared_roots);
	spin_unlock(&b->lock);
	spin_unlock_irqrestore(&a->lock, flags);
	ret = -ENOMEM;
	object = kzalloc(sizeof(*object), GFP_KERNEL);
	if (!object)
		goto unlock;
	object->backing = alloc_pages_exact(size, GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN);
	if (!object->backing) {
		kfree(object);
		goto unlock;
	}
	object->phys = virt_to_phys(object->backing);
	object->dva = DCP_DART_ALIAS + root * DCP_DART_SLICE;
	object->size = size;
	object->frontend = b;
	if ((object->phys & (PAGE_SIZE - 1)) || object->phys >= BIT_ULL(42) ||
	    size > BIT_ULL(42) - object->phys) {
		free_pages_exact(object->backing, size);
		kfree(object);
		ret = -ERANGE;
		goto unlock;
	}
	/* The cached direct alias is never touched again. Firmware owns the bytes. */
	arch_dma_prep_coherent(virt_to_page(object->backing), size);
	list_add_tail(&object->link, &a->shared);
	a->shared_count++;
	a->allocated += size;
	b->allocated += size;
	/* Backing and paired claims are permanently owned before the first PTE. */
	ret = apple_dart_dcp_map_owned(a, object->dva, object->phys, PAGE_SIZE,
				       size / PAGE_SIZE, IOMMU_READ | IOMMU_WRITE,
				       GFP_KERNEL, &mapped, true);
	if (!ret && mapped != size)
		ret = -EIO;
	if (!ret)
		ret = apple_dart_domain_flush_tlb(a->domain);
	if (!ret)
		ret = apple_dart_dcp_shared_verify(a, object);
	if (!ret)
		ret = apple_dart_dcp_check(dev);
	if (!ret)
		ret = apple_dart_dcp_check(piodma);
	if (ret) {
		WRITE_ONCE(a->failed, true);
		WRITE_ONCE(b->failed, true);
		dev_err(dev, "Retaining shared DCP backing after SID23 publication failure: %d\n", ret);
		goto unlock;
	}
	*phys = object->phys;
	*dva = object->dva;
unlock:
	mutex_unlock(&b->alloc_lock);
	mutex_unlock(&a->alloc_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_dart_dcp_shared_alloc);

int apple_dart_dcp_shared_map(struct device *dev, struct device *piodma,
			      u64 phys, u64 dva, size_t size)
{
	struct apple_dart_dcp *a, *b;
	struct apple_dart_dcp_shared *object, *found = NULL;
	size_t mapped = 0;
	int ret = apple_dart_dcp_shared_pair(dev, piodma, &a, &b);

	if (ret)
		return ret;
	mutex_lock(&a->alloc_lock);
	mutex_lock_nested(&b->alloc_lock, SINGLE_DEPTH_NESTING);
	list_for_each_entry(object, &a->shared, link)
		if (object->dva == dva) {
			found = object;
			break;
		}
	ret = -EINVAL;
	if (!found || found->frontend != b || found->phys != phys || found->size != size || found->mapped)
		goto unlock;
	ret = apple_dart_dcp_shared_verify(a, found);
	if (ret)
		goto failed;
	ret = apple_dart_dcp_map_owned(b, dva, phys, PAGE_SIZE, size / PAGE_SIZE,
				       IOMMU_READ | IOMMU_WRITE, GFP_KERNEL, &mapped, true);
	if (!ret && mapped != size)
		ret = -EIO;
	if (!ret)
		ret = apple_dart_dcp_shared_flush(b);
	if (!ret)
		ret = apple_dart_dcp_shared_verify(b, found);
	if (!ret)
		ret = apple_dart_dcp_check(dev);
	if (!ret)
		ret = apple_dart_dcp_check(piodma);
	if (ret)
		goto failed;
	found->mapped = true;
	WRITE_ONCE(b->shared_mapping, true);
	goto unlock;
failed:
	WRITE_ONCE(a->failed, true);
	WRITE_ONCE(b->failed, true);
	dev_err(dev, "Retaining shared DCP backing after SID16 publication failure: %d\n", ret);
unlock:
	mutex_unlock(&b->alloc_lock);
	mutex_unlock(&a->alloc_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_dart_dcp_shared_map);
